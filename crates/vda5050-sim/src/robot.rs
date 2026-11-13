//! The virtual robot: a VDA 5050 state machine driven by a tick clock.

use rand::Rng;
use rand::SeedableRng;
use rand_chacha::ChaCha12Rng;
use serde_json::{json, Value};
use tracing::warn;
use vda5050_types::types::{
    Action, ActionParameter, ActionState, ActionStatus, Connection, ConnectionState, EdgeState,
    ErrorEntry, ErrorLevel, InstantActions, Node, NodeState, OperatingMode, Order, PowerSupply,
    SafetyState, State, Visualization,
};
use vda5050_types::SPEC_VERSION;

use crate::config::{Profile, RobotSpec};
use crate::world::MapGraph;

/// What the robot is currently doing.
#[derive(Debug, Clone)]
enum Phase {
    /// No active order.
    Idle,
    /// Executing the actions of `order.nodes[node_index]`.
    AtNode {
        node_index: usize,
        action_index: usize,
    },
    /// Travelling towards `order.nodes[node_index]`.
    Moving {
        node_index: usize,
        remaining_s: f64,
        total_s: f64,
        /// Route polyline in map coordinates; last point is the target node.
        route: Vec<(f64, f64)>,
    },
    /// Executing `order.nodes[node_index].actions[action_index]`.
    Acting {
        node_index: usize,
        action_index: usize,
        remaining_s: f64,
    },
}

/// A virtual robot speaking VDA 5050.
#[derive(Debug)]
pub struct RobotVm {
    spec: RobotSpec,
    map: MapGraph,
    order: Option<Order>,
    order_nodes: Vec<Node>,
    phase: Phase,
    position: vda5050_types::types::Position,
    battery_percent: f64,
    charging: bool,
    paused: bool,
    driving: bool,
    operating_mode: OperatingMode,
    errors: Vec<ErrorEntry>,
    instant_action_states: Vec<ActionState>,
    header_seq: u64,
    rng: ChaCha12Rng,
    /// Seconds since start.
    elapsed_s: f64,
    /// Time until the next `state` publication.
    publish_countdown_s: f64,
    last_node_id: String,
    last_node_sequence_id: u32,
    /// Node the robot is currently at (used to compute the first leg of an order).
    current_node_id: String,
}

impl RobotVm {
    /// Create a robot at the given start node.
    #[must_use]
    pub fn new(spec: RobotSpec, map: MapGraph, start_node: &str, seed: u64) -> Self {
        let position = map
            .position(start_node)
            .unwrap_or_else(|| default_position(map.map_id.as_str()));
        Self {
            spec,
            map,
            order: None,
            order_nodes: Vec::new(),
            phase: Phase::Idle,
            position,
            battery_percent: 90.0,
            charging: false,
            paused: false,
            driving: false,
            operating_mode: OperatingMode::AUTOMATIC,
            errors: Vec::new(),
            instant_action_states: Vec::new(),
            header_seq: 0,
            rng: ChaCha12Rng::seed_from_u64(seed),
            elapsed_s: 0.0,
            publish_countdown_s: 0.0,
            last_node_id: start_node.to_owned(),
            last_node_sequence_id: 0,
            current_node_id: start_node.to_owned(),
        }
    }

    /// `headerId` is an integer counter per the v3.0.0 schema.
    fn next_header_id(&mut self) -> u64 {
        self.header_seq += 1;
        self.header_seq
    }

    fn timestamp(&self) -> String {
        // Deterministic, monotonic, millisecond precision (v3 requirement).
        #[allow(
            clippy::cast_possible_truncation,
            reason = "simulated time is bounded and positive"
        )]
        let ms = (self.elapsed_s * 1_000.0).round() as i64;
        let secs = ms.div_euclid(1000);
        let millis = ms.rem_euclid(1000);
        let (h, rem) = (secs / 3600, secs % 3600);
        let (m, s) = (rem / 60, rem % 60);
        format!("2026-01-01T{h:02}:{m:02}:{s:02}.{millis:03}Z")
    }

    /// Advance the simulation by `dt` seconds.
    pub fn tick(&mut self, dt: f64) {
        self.elapsed_s += dt;

        if let Some(order) = self.order.clone() {
            if !self.paused {
                self.advance_order(&order, dt);
            }
        } else {
            self.driving = false;
        }

        // Battery model.
        let drain = if self.driving {
            self.spec.battery.drain_w * dt / 3600.0
        } else {
            1.0 * dt / 3600.0
        };
        if self.charging {
            self.battery_percent += self.spec.battery.charge_w * dt / 3600.0;
        } else {
            self.battery_percent -= drain;
        }
        self.battery_percent = self.battery_percent.clamp(0.0, 100.0);

        self.publish_countdown_s -= dt;
    }

    fn advance_order(&mut self, order: &Order, dt: f64) {
        let phase = self.phase.clone();
        let next_phase = match phase {
            Phase::Idle => Phase::AtNode {
                node_index: 0,
                action_index: 0,
            },
            Phase::AtNode {
                node_index,
                action_index,
            } => {
                let node = &order.nodes[node_index];
                if action_index < node.actions.len() {
                    let action = &node.actions[action_index];
                    let duration = action_duration(action);
                    self.begin_action(node_index, action_index, action);
                    Phase::Acting {
                        node_index,
                        action_index,
                        remaining_s: duration,
                    }
                } else {
                    self.depart_node(order, node_index)
                }
            }
            Phase::Acting {
                node_index,
                action_index,
                remaining_s,
            } => {
                let remaining = remaining_s - dt;
                if remaining > 0.0 {
                    Phase::Acting {
                        node_index,
                        action_index,
                        remaining_s: remaining,
                    }
                } else {
                    self.finish_action(order, node_index, action_index);
                    Phase::AtNode {
                        node_index,
                        action_index: action_index + 1,
                    }
                }
            }
            Phase::Moving {
                node_index,
                remaining_s,
                total_s,
                route,
            } => {
                let remaining = (remaining_s - dt).max(0.0);
                let progress = if total_s > 0.0 {
                    (1.0 - remaining / total_s).clamp(0.0, 1.0)
                } else {
                    1.0
                };
                self.position = position_along(&route, progress, &self.position);
                self.driving = true;
                if remaining > 0.0 {
                    Phase::Moving {
                        node_index,
                        remaining_s: remaining,
                        total_s,
                        route,
                    }
                } else {
                    self.arrive_at_node(order, node_index);
                    Phase::AtNode {
                        node_index,
                        action_index: 0,
                    }
                }
            }
        };
        self.phase = next_phase;
    }

    fn begin_action(&mut self, _node_index: usize, _action_index: usize, action: &Action) {
        self.driving = false;
        if action.action_type == "charge" {
            self.charging = true;
        }
    }

    fn finish_action(&mut self, order: &Order, node_index: usize, action_index: usize) {
        let action = &order.nodes[node_index].actions[action_index];
        if action.action_type == "charge" {
            self.charging = false;
        }
        let failed = self.spec.faults.action_fail_prob > 0.0
            && self.rng.random_bool(self.spec.faults.action_fail_prob);
        self.errors.retain(|e| e.error_type != "ACTION_FAILED");
        if failed {
            self.errors.push(ErrorEntry {
                error_type: "ACTION_FAILED".to_owned(),
                error_references: vec![json!(action.action_id)],
                error_description: Some("simulated action failure".to_owned()),
                error_hint: Some("fault injection".to_owned()),
                error_level: ErrorLevel::WARNING,
            });
        }
    }

    fn depart_node(&mut self, order: &Order, node_index: usize) -> Phase {
        let Some(next_index) = node_index.checked_add(1) else {
            return self.complete_order(order);
        };
        if next_index >= order.nodes.len() {
            return self.complete_order(order);
        }
        let from_id = order.nodes[node_index].node_id.clone();
        let to_id = order.nodes[next_index].node_id.clone();
        let total = self.map.travel_time(&from_id, &to_id);
        // 직통 엣지가 없어도 최단 경로(통로 경유)로 이동한다.
        let mut route = self.map.polyline(&from_id, &to_id);
        if route.len() < 2 {
            warn!(from = %from_id, to = %to_id, "맵에 경로가 없어 제자리에 머무릅니다");
            route = vec![
                (self.position.x, self.position.y),
                (self.position.x, self.position.y),
            ];
        } else {
            route[0] = (self.position.x, self.position.y);
        }
        self.driving = total > 0.0;
        Phase::Moving {
            node_index: next_index,
            remaining_s: total,
            total_s: total,
            route,
        }
    }

    fn arrive_at_node(&mut self, order: &Order, node_index: usize) {
        let node = &order.nodes[node_index];
        if let Some(p) = self.map.position(&node.node_id) {
            self.position = p;
        }
        self.driving = false;
        self.last_node_id.clone_from(&node.node_id);
        self.last_node_sequence_id = node.sequence_id;
    }

    fn complete_order(&mut self, order: &Order) -> Phase {
        if let Some(last) = order.nodes.last() {
            self.last_node_id.clone_from(&last.node_id);
            self.last_node_sequence_id = last.sequence_id;
        }
        self.order = None;
        self.order_nodes = Vec::new();
        self.driving = false;
        Phase::Idle
    }

    /// Try to accept an order. Returns `false` when the robot rejects it.
    pub fn accept_order(&mut self, order: Order) -> bool {
        // Reject unknown action types, like a real fleet would.
        if let Some(unsupported) = order
            .nodes
            .iter()
            .flat_map(|n| n.actions.iter())
            .chain(order.edges.iter().flat_map(|e| e.actions.iter()))
            .find(|a| !self.spec.actions.contains(&a.action_type))
        {
            self.errors.push(ErrorEntry {
                error_type: "INVALID_ORDER_ACTION".to_owned(),
                error_references: vec![json!(order.order_id)],
                error_description: Some(format!("unsupported action {}", unsupported.action_type)),
                error_hint: Some("check factsheet protocolFeatures".to_owned()),
                error_level: ErrorLevel::WARNING,
            });
            return false;
        }

        let reject = self.spec.faults.reject_order_prob > 0.0
            && self.rng.random_bool(self.spec.faults.reject_order_prob);
        if reject {
            self.errors.push(ErrorEntry {
                error_type: "VALIDATION_FAILURE".to_owned(),
                error_references: vec![json!(order.order_id)],
                error_description: Some("simulated order rejection".to_owned()),
                error_hint: Some("fault injection".to_owned()),
                error_level: ErrorLevel::WARNING,
            });
            return false;
        }

        // The robot must travel to the first node before acting on it.
        let first_node = order
            .nodes
            .first()
            .map(|n| (n.node_id.clone(), n.sequence_id));
        let already_there = first_node
            .as_ref()
            .and_then(|(id, _)| self.map.position(id))
            .is_some_and(|p| {
                (p.x - self.position.x).abs() < 0.05 && (p.y - self.position.y).abs() < 0.05
            });

        self.order_nodes.clone_from(&order.nodes);
        self.order = Some(order);
        self.phase = match first_node {
            Some((node_id, _)) if !already_there => {
                let total = self.map.travel_time(&self.current_node_id, &node_id);
                let mut route = self.map.polyline(&self.current_node_id, &node_id);
                if route.len() < 2 {
                    route = vec![
                        (self.position.x, self.position.y),
                        (self.position.x, self.position.y),
                    ];
                } else {
                    route[0] = (self.position.x, self.position.y);
                }
                Phase::Moving {
                    node_index: 0,
                    remaining_s: total,
                    total_s: total,
                    route,
                }
            }
            _ => Phase::AtNode {
                node_index: 0,
                action_index: 0,
            },
        };
        self.instant_action_states.clear();
        self.errors.retain(|e| e.error_type != "ACTION_FAILED");
        true
    }

    /// Apply an instant action. Returns the recorded action status.
    pub fn apply_instant_action(&mut self, actions: &InstantActions) -> Vec<ActionState> {
        let mut recorded = Vec::new();
        for action in &actions.actions {
            match action.action_type.as_str() {
                "startPause" => {
                    self.paused = true;
                    self.driving = false;
                    recorded.push(done(action));
                }
                "stopPause" => {
                    self.paused = false;
                    recorded.push(done(action));
                }
                "cancelOrder" => {
                    self.cancel_order();
                    recorded.push(done(action));
                }
                "startHibernation" => {
                    self.operating_mode = OperatingMode::SERVICE;
                    recorded.push(done(action));
                }
                "stopHibernation" => {
                    self.operating_mode = OperatingMode::AUTOMATIC;
                    recorded.push(done(action));
                }
                "updateCertificate" | "trigger" => recorded.push(done(action)),
                other => {
                    self.errors.push(ErrorEntry {
                        error_type: "UNSUPPORTED_ACTION".to_owned(),
                        error_references: vec![json!(action.action_id)],
                        error_description: Some(format!("unsupported instant action {other}")),
                        error_hint: None,
                        error_level: ErrorLevel::WARNING,
                    });
                    recorded.push(ActionState {
                        action_id: action.action_id.clone(),
                        action_type: Some(action.action_type.clone()),
                        action_descriptor: action.action_descriptor.clone(),
                        action_status: ActionStatus::FAILED,
                        action_result: Some("unsupported".to_owned()),
                    });
                }
            }
        }
        self.instant_action_states.clone_from(&recorded);
        recorded
    }

    fn cancel_order(&mut self) {
        self.order = None;
        self.order_nodes = Vec::new();
        self.phase = Phase::Idle;
        self.driving = false;
        self.paused = false;
        self.last_node_id = String::new();
        self.last_node_sequence_id = 0;
    }

    /// Current `state` message.
    #[must_use]
    pub fn state_message(&mut self) -> State {
        let header_id = self.next_header_id();
        let (node_states, edge_states) = self.node_and_edge_states();
        State {
            header_id,
            timestamp: self.timestamp(),
            version: SPEC_VERSION.to_owned(),
            manufacturer: self.spec.manufacturer.clone(),
            serial_number: self.spec.serial_number.clone(),
            maps: vec![json!({
                "mapId": self.map.map_id.as_str(),
                "mapVersion": "1.0.0",
                "mapStatus": "ENABLED"
            })],
            zone_sets: Vec::new(),
            order_id: self
                .order
                .as_ref()
                .map_or(String::new(), |o| o.order_id.clone()),
            order_update_id: self.order.as_ref().map_or(0, |o| o.order_update_id),
            last_node_id: self.last_node_id.clone(),
            last_node_sequence_id: self.last_node_sequence_id,
            node_states,
            edge_states,
            planned_path: self.planned_path(),
            intermediate_path: None,
            mobile_robot_position: Some(self.position.clone()),
            velocity: Some(json!({ "vx": 0.0, "vy": 0.0, "omega": 0.0 })),
            loads: Vec::new(),
            driving: self.driving,
            paused: self.paused,
            new_base_request: false,
            zone_requests: Vec::new(),
            edge_requests: Vec::new(),
            distance_since_last_node: None,
            action_states: self.action_states(),
            instant_action_states: self.instant_action_states.clone(),
            zone_action_states: Vec::new(),
            power_supply: PowerSupply {
                state_of_charge: self.battery_percent,
                charging: self.charging,
                battery_voltage: Some(48.0),
                battery_current: Some(if self.charging { 6.0 } else { -1.5 }),
                battery_health: Some(1.0),
                range: None,
            },
            operating_mode: self.operating_mode,
            errors: self.errors.clone(),
            information: Vec::new(),
            safety_state: SafetyState {
                active_emergency_stop: vda5050_types::types::EmergencyStop::NONE,
                field_violation: false,
            },
        }
    }

    fn node_and_edge_states(&self) -> (Vec<NodeState>, Vec<EdgeState>) {
        let Some(order) = self.order.as_ref() else {
            return (Vec::new(), Vec::new());
        };
        let current_index = match self.phase {
            Phase::AtNode { node_index, .. }
            | Phase::Moving { node_index, .. }
            | Phase::Acting { node_index, .. } => node_index,
            Phase::Idle => usize::MAX,
        };
        let nodes = order
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| NodeState {
                node_id: n.node_id.clone(),
                sequence_id: n.sequence_id,
                node_descriptor: n.node_descriptor.clone(),
                released: i <= current_index,
                node_position: n
                    .node_position
                    .clone()
                    .or_else(|| self.map.position(&n.node_id)),
            })
            .collect();
        let edges = order
            .edges
            .iter()
            .enumerate()
            .map(|(i, e)| EdgeState {
                edge_id: e.edge_id.clone(),
                sequence_id: e.sequence_id,
                edge_descriptor: e.edge_descriptor.clone(),
                released: i < current_index,
            })
            .collect();
        (nodes, edges)
    }

    fn action_states(&self) -> Vec<ActionState> {
        let Some(order) = self.order.as_ref() else {
            return Vec::new();
        };
        let (node_index, action_index) = match self.phase {
            Phase::AtNode {
                node_index,
                action_index,
            }
            | Phase::Acting {
                node_index,
                action_index,
                ..
            } => (node_index, action_index),
            Phase::Moving { node_index, .. } => {
                // Still travelling: nothing at the target node has started yet.
                (node_index, 0)
            }
            Phase::Idle => return Vec::new(),
        };
        let Some(node) = order.nodes.get(node_index) else {
            return Vec::new();
        };
        node.actions
            .iter()
            .enumerate()
            .map(|(i, a)| ActionState {
                action_id: a.action_id.clone(),
                action_type: Some(a.action_type.clone()),
                action_descriptor: a.action_descriptor.clone(),
                action_status: match i.cmp(&action_index) {
                    std::cmp::Ordering::Less => ActionStatus::FINISHED,
                    std::cmp::Ordering::Equal if matches!(self.phase, Phase::Acting { .. }) => {
                        ActionStatus::RUNNING
                    }
                    _ => ActionStatus::WAITING,
                },
                action_result: None,
            })
            .collect()
    }

    fn planned_path(&self) -> Option<Value> {
        let order = self.order.as_ref()?;
        let remaining: Vec<Value> = order
            .nodes
            .iter()
            .filter_map(|n| self.map.position(&n.node_id))
            .map(|p| json!({ "x": p.x, "y": p.y, "theta": p.theta, "mapId": p.map_id }))
            .collect();
        Some(json!({
            "trajectory": { "degree": 1, "knotVector": [], "controlPoints": remaining },
            "traversedNodes": []
        }))
    }

    /// `visualization` message.
    #[must_use]
    pub fn visualization_message(&mut self, state_header_id: u64) -> Visualization {
        Visualization {
            header_id: self.next_header_id(),
            timestamp: self.timestamp(),
            version: SPEC_VERSION.to_owned(),
            manufacturer: self.spec.manufacturer.clone(),
            serial_number: self.spec.serial_number.clone(),
            reference_state_header_id: state_header_id.to_string(),
            planned_path: self.planned_path(),
            intermediate_path: None,
            mobile_robot_position: Some(self.position.clone()),
            velocity: Some(json!({ "vx": 0.0, "vy": 0.0, "omega": 0.0 })),
        }
    }

    /// `connection` message.
    #[must_use]
    pub fn connection_message(&mut self, state: ConnectionState) -> Connection {
        Connection {
            header_id: self.next_header_id(),
            timestamp: self.timestamp(),
            version: SPEC_VERSION.to_owned(),
            manufacturer: self.spec.manufacturer.clone(),
            serial_number: self.spec.serial_number.clone(),
            connection_state: state,
        }
    }

    /// `factsheet` message describing this virtual robot.
    #[must_use]
    pub fn factsheet_message(&mut self) -> Value {
        let actions: Vec<Value> = self
            .spec
            .actions
            .iter()
            .map(|action| {
                let scopes = if matches!(
                    action.as_str(),
                    "startPause" | "stopPause" | "cancelOrder" | "updateCertificate" | "trigger"
                ) {
                    json!(["INSTANT"])
                } else {
                    json!(["NODE", "EDGE"])
                };
                json!({
                    "actionType": action,
                    "actionDescription": "simulated action",
                    "actionScopes": scopes,
                    "blockingTypes": ["HARD", "SOFT", "NONE"],
                    // The schema types these as strings, not booleans.
                    "pauseAllowed": "true",
                    "cancelAllowed": "true",
                    "actionParameters": []
                })
            })
            .collect();

        // The partial profile does not implement `zoneSet`, so it does not claim it.
        let optional_parameters = if self.spec.profile == Profile::V3Partial {
            json!([{ "parameter": "node.nodePosition.allowedDeviationTheta", "support": "SUPPORTED" }])
        } else {
            json!([
                { "parameter": "node.nodePosition.allowedDeviationTheta", "support": "SUPPORTED" },
                { "parameter": "zoneSet", "support": "SUPPORTED" }
            ])
        };

        json!({
            "headerId": self.next_header_id(),
            "timestamp": self.timestamp(),
            "version": SPEC_VERSION,
            "manufacturer": self.spec.manufacturer,
            "serialNumber": self.spec.serial_number,
            "typeSpecification": {
                "seriesName": format!("{}-{}", self.spec.fleet, self.spec.serial_number),
                "seriesDescription": "vda5050-sim virtual robot",
                "mobileRobotKinematic": "DIFFERENTIAL",
                "mobileRobotClass": "PLATFORM",
                "maximumLoadMass": 100.0,
                "localizationTypes": ["GRID"],
                "navigationTypes": ["PHYSICAL_LINE_GUIDED"]
            },
            "physicalParameters": {
                "minimumSpeed": 0.0,
                "maximumSpeed": 1.5,
                "maximumAcceleration": 0.5,
                "maximumDeceleration": 0.5,
                "minimumHeight": 0.1,
                "maximumHeight": 0.4,
                "width": 0.6,
                "length": 0.8
            },
            "protocolLimits": {
                "maximumStringLengths": {
                    "maximumMessageLength": 100_000,
                    "maximumTopicSerialLength": 64,
                    "maximumTopicElementLength": 64
                },
                "maximumArrayLengths": {
                    "orderNodes": 64, "orderEdges": 64, "nodeActions": 8, "edgeActions": 8,
                    "actionParameters": 8, "instantActions": 8, "nodeStates": 64,
                    "edgeStates": 64, "actionStates": 64, "errors": 16, "informations": 8, "loads": 8
                },
                "timing": { "minimumOrderInterval": 0.0, "minimumStateInterval": 0.1 }
            },
            "protocolFeatures": {
                "optionalParameters": optional_parameters,
                "mobileRobotActions": actions
            },
            "mobileRobotGeometry": {
                "footprint": { "x": 0.8, "y": 0.6, "z": 0.3 },
                "heightMax": 0.4,
                "heightMin": 0.1,
                "envelopes": []
            },
            "loadSpecification": {
                "loadPositions": ["lhd-1"]
            },
            "mobileRobotConfiguration": {
                "versions": [ { "key": "softwareVersion", "value": "vda5050-sim-0.1.0" } ]
            }
        })
    }

    /// Serialise the current state as v2 JSON (used by the `v2-only` profile).
    #[must_use]
    pub fn state_as_v2_value(&mut self) -> Value {
        let state = self.state_message();
        let agv_position = state.mobile_robot_position.map(|p| {
            json!({
                "x": p.x, "y": p.y, "theta": p.theta, "mapId": p.map_id,
                "positionInitialized": p.localized,
                "localizationScore": p.localization_score
            })
        });
        json!({
            // v2 nests the header fields in a `header` object.
            "header": {
                "headerId": state.header_id,
                "timestamp": state.timestamp,
                "version": "2.1.0",
                "manufacturer": state.manufacturer,
                "serialNumber": state.serial_number
            },
            "version": "2.1.0",
            "manufacturer": state.manufacturer,
            "serialNumber": state.serial_number,
            "orderId": state.order_id,
            "orderUpdateId": state.order_update_id,
            "lastNodeId": state.last_node_id,
            "lastNodeSequenceId": state.last_node_sequence_id,
            "nodeStates": state.node_states,
            "edgeStates": state.edge_states,
            "driving": state.driving,
            "agvPosition": agv_position,
            "velocity": state.velocity,
            "loads": state.loads,
            "paused": state.paused,
            "actionStates": state.action_states.iter().map(|a| json!({
                "actionId": a.action_id,
                "actionType": a.action_type,
                "actionDescription": a.action_descriptor,
                "status": a.action_status,
                "resultDescription": a.action_result
            })).collect::<Vec<_>>(),
            "batteryState": {
                "batteryCharge": state.power_supply.state_of_charge,
                "batteryVoltage": state.power_supply.battery_voltage,
                "charging": state.power_supply.charging,
                "reach": state.power_supply.range
            },
            "operatingMode": state.operating_mode,
            "errors": state.errors.iter().map(|e| json!({
                "errorType": e.error_type,
                "errorReferences": e.error_references,
                "errorDescription": e.error_description,
                "errorLevel": e.error_level
            })).collect::<Vec<_>>(),
            "safetyState": {
                "eStop": "NONE",
                "fieldViolation": false
            }
        })
    }

    /// Seconds of simulated time elapsed since start.
    #[must_use]
    pub const fn elapsed_seconds(&self) -> f64 {
        self.elapsed_s
    }

    /// Id of the order currently being executed, if any.
    #[must_use]
    pub fn current_order_id(&self) -> String {
        self.order
            .as_ref()
            .map_or(String::new(), |o| o.order_id.clone())
    }

    /// Whether a `state` should be published now.
    #[must_use]
    pub fn should_publish(&self) -> bool {
        self.publish_countdown_s <= 0.0
    }

    /// Reset the publish countdown.
    pub fn mark_published(&mut self) {
        self.publish_countdown_s = 1.0 / self.spec.publish_hz.max(0.1);
    }

    /// True when the robot should stop publishing (fault injection).
    #[must_use]
    pub fn is_silent(&self) -> bool {
        self.spec
            .faults
            .go_silent_after_s
            .is_some_and(|t| self.elapsed_s >= t)
    }
}

/// Position at fraction `t` (0..1) along a polyline, keeping the current
/// position's metadata. `t >= 1` snaps to the last point.
fn position_along(
    route: &[(f64, f64)],
    t: f64,
    current: &vda5050_types::types::Position,
) -> vda5050_types::types::Position {
    let t = t.clamp(0.0, 1.0);
    if route.len() < 2 {
        return current.clone();
    }
    let total: f64 = route
        .windows(2)
        .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
        .sum();
    if total <= 0.0 {
        return current.clone();
    }
    let mut target = total * t;
    for w in route.windows(2) {
        let seg = (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1);
        if target > seg {
            target -= seg;
            continue;
        }
        let f = if seg > 0.0 { target / seg } else { 0.0 };
        return vda5050_types::types::Position {
            x: w[0].0 + (w[1].0 - w[0].0) * f,
            y: w[0].1 + (w[1].1 - w[0].1) * f,
            theta: (w[1].1 - w[0].1).atan2(w[1].0 - w[0].0),
            map_id: current.map_id.clone(),
            localized: true,
            localization_score: current.localization_score,
            deviation_range: current.deviation_range,
        };
    }
    let (x, y) = route.last().copied().unwrap_or((current.x, current.y));
    vda5050_types::types::Position {
        x,
        y,
        theta: current.theta,
        map_id: current.map_id.clone(),
        localized: true,
        localization_score: current.localization_score,
        deviation_range: current.deviation_range,
    }
}

fn default_position(map_id: &str) -> vda5050_types::types::Position {
    vda5050_types::types::Position {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
        map_id: map_id.to_owned(),
        localized: false,
        localization_score: None,
        deviation_range: None,
    }
}

fn done(action: &vda5050_types::types::InstantAction) -> ActionState {
    ActionState {
        action_id: action.action_id.clone(),
        action_type: Some(action.action_type.clone()),
        action_descriptor: action.action_descriptor.clone(),
        action_status: ActionStatus::FINISHED,
        action_result: Some("SUCCESS".to_owned()),
    }
}

fn action_duration(action: &Action) -> f64 {
    action
        .action_parameters
        .iter()
        .find(|p| p.key == "duration")
        .and_then(|p: &ActionParameter| p.value.as_f64())
        .unwrap_or(2.0)
}
