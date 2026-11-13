//! Mission issuing: turn an operator request into a schema-valid VDA 5050
//! `order` and publish it on the robot's own topic.
//!
//! The dashboard submits a *route* (waypoints plus actions); everything the
//! operator should not have to memorise is derived here:
//!
//! * `headerId` — monotonic integer, seeded from the wall clock so a restarted
//!   gateway does not reuse ids a robot has already seen.
//! * `sequenceId` — nodes get even ids, edges the odd ids in between.
//! * `nodePosition` / edge `length` — looked up from the served map, so the
//!   order carries the geometry the robot needs to plan.
//! * `released` — `true`, so a freshly submitted mission starts driving.

use std::collections::{BTreeMap, HashSet};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use tracing::info;
use vda5050_types::types::{
    Action, ActionParameter, BlockingType, Edge, InstantAction, InstantActions, Node, Order,
    Position,
};
use vda5050_types::{MessageValidator, Topic, TopicPath};

use crate::store::Store;

/// One action the robot should run at a waypoint.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ActionSpec {
    /// VDA 5050 action type, e.g. `pick`, `drop`, `charge`, `initPosition`.
    pub action_type: String,
    /// Operator-facing label; generated from the position when omitted.
    #[serde(default)]
    pub action_id: Option<String>,
    /// How long the action takes, in seconds.
    #[serde(default)]
    pub duration_s: Option<f64>,
    /// `NONE` | `SOFT` | `SINGLE` | `HARD`. Defaults to `HARD`, which is what
    /// makes the robot finish the action before it drives on.
    #[serde(default)]
    pub blocking_type: Option<String>,
    /// Extra action parameters, passed through verbatim.
    #[serde(default)]
    pub parameters: BTreeMap<String, Value>,
}

/// A single stop of the mission.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Waypoint {
    /// Node id on the served map.
    pub node_id: String,
    /// Actions to run once the robot reaches the node.
    #[serde(default)]
    pub actions: Vec<ActionSpec>,
    /// `false` keeps the waypoint unreleased, so the robot waits for a later
    /// `orderUpdate` before continuing.
    #[serde(default)]
    pub released: Option<bool>,
}

/// The mission as submitted by the dashboard.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct OrderRequest {
    pub manufacturer: String,
    pub serial_number: String,
    pub order_id: String,
    #[serde(default)]
    pub order_update_id: u32,
    #[serde(default)]
    pub order_description: Option<String>,
    /// At least one waypoint, in travel order.
    pub nodes: Vec<Waypoint>,
}

/// Assembles [`OrderRequest`]s into wire-format orders.
///
/// Holds the map (for geometry) and the `headerId` sequence.
#[derive(Debug)]
pub struct OrderAssembler<'a> {
    map: &'a Value,
    /// Seeded from the wall clock so ids stay unique across gateway restarts.
    header_seq: AtomicU64,
}

impl<'a> OrderAssembler<'a> {
    /// Create an assembler for one served map.
    #[must_use]
    pub fn new(map: &'a Value) -> Self {
        let seed = u64::try_from(
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(1, |d| d.as_millis()),
        )
        .unwrap_or(1);
        Self {
            map,
            header_seq: AtomicU64::new(seed),
        }
    }

    /// Build a schema-valid order, or explain every problem at once.
    ///
    /// # Errors
    /// Returns one entry per problem so the operator can fix them all in one
    /// pass instead of discovering them one at a time.
    pub fn assemble(&self, request: &OrderRequest) -> Result<Order, Vec<String>> {
        if let Some(problems) = self.check(request) {
            return Err(problems);
        }
        Ok(self.build(request))
    }

    /// Every problem with the request, or `None` when it is usable.
    fn check(&self, request: &OrderRequest) -> Option<Vec<String>> {
        let mut problems = Vec::new();

        // A `/` in either identity field would escape its topic level.
        for (field, value) in [
            ("manufacturer", &request.manufacturer),
            ("serialNumber", &request.serial_number),
        ] {
            if value.trim().is_empty() {
                problems.push(format!("{field} must not be empty"));
            } else if value.contains('/') {
                problems.push(format!("{field} must not contain '/'"));
            }
        }
        if request.order_id.trim().is_empty() {
            problems.push("orderId must not be empty".to_owned());
        }
        if request.nodes.is_empty() {
            problems.push("a mission needs at least one waypoint".to_owned());
        }

        let mut action_ids: HashSet<String> = HashSet::new();
        // Must mirror the numbering in `build`, or two auto-generated ids would
        // be reported as duplicates while the order itself is fine.
        let mut auto_action = 0_usize;
        for (index, waypoint) in request.nodes.iter().enumerate() {
            if waypoint.node_id.trim().is_empty() {
                problems.push(format!("waypoint {index} has no nodeId"));
            } else if self.node_position(&waypoint.node_id).is_none() {
                problems.push(format!(
                    "waypoint {index}: node '{}' is not on map {}",
                    waypoint.node_id,
                    self.map_id()
                ));
            }
            for action in &waypoint.actions {
                if action.action_type.trim().is_empty() {
                    problems.push(format!("waypoint {index}: action needs an actionType"));
                }
                if let Some(duration) = action.duration_s {
                    if !duration.is_finite() || duration < 0.0 {
                        problems.push(format!(
                            "waypoint {index}: duration must be a non-negative number"
                        ));
                    }
                }
                auto_action += 1;
                let label = action
                    .action_id
                    .clone()
                    .unwrap_or_else(|| format!("a{auto_action}"));
                if !action_ids.insert(label.clone()) {
                    problems.push(format!("duplicate actionId '{label}'"));
                }
            }
        }

        if problems.is_empty() {
            return None;
        }
        Some(problems)
    }

    /// Turn a checked request into a wire-format order.
    ///
    /// Assumes [`Self::check`] passed: the caller reports problems, so this only
    /// has to lay out sequence ids and geometry.
    fn build(&self, request: &OrderRequest) -> Order {
        let header_id = self.header_seq.fetch_add(1, Ordering::SeqCst);

        // Nodes take the even sequence ids, edges the odd ones in between, so
        // the order is `node, edge, node, edge, ...`.
        let mut nodes = Vec::with_capacity(request.nodes.len());
        let mut edges = Vec::with_capacity(request.nodes.len().saturating_sub(1));
        let mut auto_action = 0_usize;
        for (index, waypoint) in request.nodes.iter().enumerate() {
            let mut actions = Vec::with_capacity(waypoint.actions.len());
            for action in &waypoint.actions {
                auto_action += 1;
                let action_id = action
                    .action_id
                    .clone()
                    .unwrap_or_else(|| format!("a{auto_action}"));
                let mut parameters: Vec<ActionParameter> = action
                    .parameters
                    .iter()
                    .map(|(key, value)| ActionParameter {
                        key: key.clone(),
                        value: value.clone(),
                    })
                    .collect();
                if let Some(duration) = action.duration_s {
                    parameters.push(ActionParameter {
                        key: "duration".to_owned(),
                        value: json!(duration),
                    });
                }
                actions.push(Action {
                    action_type: action.action_type.clone(),
                    action_id,
                    action_descriptor: None,
                    blocking_type: parse_blocking(action.blocking_type.as_deref()),
                    action_parameters: parameters,
                    retriable: None,
                });
            }
            nodes.push(Node {
                node_id: waypoint.node_id.clone(),
                sequence_id: u32::try_from(index * 2).unwrap_or(u32::MAX),
                node_descriptor: None,
                released: waypoint.released.unwrap_or(true),
                node_position: self.node_position(&waypoint.node_id),
                actions,
            });
            if let Some(next) = request.nodes.get(index + 1) {
                edges.push(Edge {
                    edge_id: format!("e{}", index + 1),
                    sequence_id: u32::try_from(index * 2 + 1).unwrap_or(u32::MAX),
                    edge_descriptor: None,
                    released: waypoint.released.unwrap_or(true),
                    maximum_speed: None,
                    orientation: None,
                    direction: None,
                    reach_orientation_before_entering: None,
                    length: self.edge_length(&waypoint.node_id, &next.node_id),
                    start_node_id: Some(waypoint.node_id.clone()),
                    end_node_id: Some(next.node_id.clone()),
                    actions: Vec::new(),
                });
            }
        }

        Order {
            header_id,
            timestamp: crate::ingest::now_string(),
            version: vda5050_types::SPEC_VERSION.to_owned(),
            manufacturer: request.manufacturer.clone(),
            serial_number: request.serial_number.clone(),
            order_id: request.order_id.clone(),
            order_update_id: request.order_update_id,
            order_description: request.order_description.clone(),
            nodes,
            edges,
        }
    }

    /// `mapId` of the served map, for error messages.
    fn map_id(&self) -> &str {
        self.map
            .get("mapId")
            .and_then(Value::as_str)
            .unwrap_or("unknown")
    }

    /// Position of a node on the served map, if it is known.
    fn node_position(&self, node_id: &str) -> Option<Position> {
        let node = self.map.get("nodes")?.get(node_id)?;
        Some(Position {
            x: node.get("x")?.as_f64()?,
            y: node.get("y")?.as_f64()?,
            theta: node.get("theta").and_then(Value::as_f64).unwrap_or(0.0),
            map_id: self.map_id().to_owned(),
            localized: true,
            localization_score: None,
            deviation_range: None,
        })
    }

    /// Length of the edge between two nodes on the served map, if it is known.
    fn edge_length(&self, from: &str, to: &str) -> Option<f64> {
        self.map.get("edges")?.as_array()?.iter().find_map(|edge| {
            let start = edge.get("from").and_then(Value::as_str)?;
            let end = edge.get("to").and_then(Value::as_str)?;
            if (start == from && end == to) || (start == to && end == from) {
                edge.get("length").and_then(Value::as_f64)
            } else {
                None
            }
        })
    }
}

/// Map the operator's choice onto the wire enum, defaulting to `HARD`.
fn parse_blocking(raw: Option<&str>) -> BlockingType {
    match raw.map(str::to_ascii_uppercase).as_deref() {
        Some("NONE") => BlockingType::NONE,
        Some("SOFT") => BlockingType::SOFT,
        Some("SINGLE") => BlockingType::SINGLE,
        _ => BlockingType::HARD,
    }
}

/// Publishes messages on a robot's own topics, validating each one against
/// the official schema before it leaves.
#[derive(Debug)]
pub struct OrderPublisher {
    client: Arc<rumqttc::AsyncClient>,
    validator: Arc<MessageValidator>,
    interface: String,
    major_version: u8,
}

impl OrderPublisher {
    /// Wrap the broker client the ingest loop already runs.
    #[must_use]
    pub fn new(
        client: Arc<rumqttc::AsyncClient>,
        validator: Arc<MessageValidator>,
        interface: String,
        major_version: u8,
    ) -> Self {
        Self {
            client,
            validator,
            interface,
            major_version,
        }
    }

    /// Validate and publish any serialisable VDA 5050 message.
    ///
    /// # Errors
    /// Returns the schema violation reasons when the message is not valid
    /// VDA 5050, or the broker error when the publish fails.
    pub async fn publish<T: Serialize>(
        &self,
        topic: Topic,
        manufacturer: &str,
        serial_number: &str,
        message: &T,
    ) -> Result<String, PublishError> {
        let value = serde_json::to_value(message).map_err(PublishError::Serialize)?;
        // Reject before the robot ever sees it: a malformed message would only
        // surface as an error entry in the robot's own state.
        let valid = self
            .validator
            .validate_value(topic, value)
            .map_err(|rejected| PublishError::Invalid(rejected.reasons))?;
        let payload = serde_json::to_vec(&valid).map_err(PublishError::Serialize)?;

        let path = TopicPath::new(
            self.interface.clone(),
            self.major_version,
            manufacturer.to_owned(),
            serial_number.to_owned(),
            topic,
        );
        self.client
            .publish(path.to_string(), rumqttc::QoS::AtLeastOnce, false, payload)
            .await
            .map_err(PublishError::Broker)?;
        Ok(path.to_string())
    }
}

/// Why a mission could not be sent.
#[derive(Debug, thiserror::Error)]
pub enum PublishError {
    #[error("assembled order is not valid VDA 5050: {0:?}")]
    Invalid(Vec<String>),
    #[error("could not serialise the order: {0}")]
    Serialize(serde_json::Error),
    #[error("broker refused the publish: {0}")]
    Broker(rumqttc::ClientError),
}

/// Validate, publish and record a mission coming from the dashboard.
///
/// Returns the topic it was published on. The caller turns failures into HTTP
/// status codes.
pub async fn submit(
    assembler: &OrderAssembler<'_>,
    publisher: &OrderPublisher,
    store: &Arc<Store>,
    request: &OrderRequest,
) -> Result<Order, Vec<String>> {
    let order = assembler.assemble(request)?;
    let topic = publisher
        .publish(
            Topic::Order,
            &order.manufacturer,
            &order.serial_number,
            &order,
        )
        .await
        .map_err(|error| vec![error.to_string()])?;
    let robot_id = format!("{}:{}", order.manufacturer, order.serial_number);
    info!(
        %topic,
        %robot_id,
        order_id = %order.order_id,
        header_id = order.header_id,
        waypoints = order.nodes.len(),
        "mission published"
    );
    store
        .record(crate::store::GatewayEvent::OrderPublished {
            robot_id,
            order_id: order.order_id.clone(),
            header_id: order.header_id,
            waypoints: order.nodes.len(),
            topic,
        })
        .await;
    Ok(order)
}

/// One instant action as submitted by the dashboard.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct InstantActionSpec {
    /// VDA 5050 instant action type, e.g. `cancelOrder`, `startPause`.
    pub action_type: String,
    /// Operator-facing label; generated when omitted.
    #[serde(default)]
    pub action_id: Option<String>,
    /// Extra action parameters, passed through verbatim.
    #[serde(default)]
    pub parameters: BTreeMap<String, Value>,
}

/// A batch of instant actions for one robot.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct InstantActionRequest {
    /// At least one action; VDA 5050 allows several per message.
    pub actions: Vec<InstantActionSpec>,
}

/// Instant actions the control screen offers.
///
/// The set a robot actually honours depends on its factsheet; the gateway does
/// not filter, because a wrong guess is reported by the robot itself as
/// `UNSUPPORTED_ACTION` and surfaces in its state.
pub const INSTANT_ACTIONS: &[&str] = &[
    "startPause",
    "stopPause",
    "cancelOrder",
    "requestRelease",
    "stateRequest",
    "startHibernation",
    "stopHibernation",
];

/// Assemble, validate and publish instant actions for one robot.
///
/// # Errors
/// Returns one reason per problem, so the operator can fix them all at once.
pub async fn submit_instant(
    publisher: &OrderPublisher,
    store: &Arc<Store>,
    robot_id: &str,
    request: &InstantActionRequest,
) -> Result<InstantActions, Vec<String>> {
    let Some((manufacturer, serial_number)) = robot_id.split_once(':') else {
        return Err(vec![format!(
            "robot id '{robot_id}' must look like manufacturer:serialNumber"
        )]);
    };
    if request.actions.is_empty() {
        return Err(vec!["at least one action is required".to_owned()]);
    }

    let mut problems = Vec::new();
    let mut seen: HashSet<String> = HashSet::new();
    for (index, action) in request.actions.iter().enumerate() {
        if action.action_type.trim().is_empty() {
            problems.push(format!("action {index}: actionType is required"));
        }
        let label = action
            .action_id
            .clone()
            .unwrap_or_else(|| format!("ia{}", index + 1));
        if !seen.insert(label.clone()) {
            problems.push(format!("duplicate actionId '{label}'"));
        }
    }
    if !problems.is_empty() {
        return Err(problems);
    }

    let actions = request
        .actions
        .iter()
        .enumerate()
        .map(|(index, action)| {
            let mut parameters: Vec<ActionParameter> = action
                .parameters
                .iter()
                .map(|(key, value)| ActionParameter {
                    key: key.clone(),
                    value: value.clone(),
                })
                .collect();
            // `cancelOrder` is the one action with a defined parameter.
            if action.action_type == "cancelOrder" && !parameters.iter().any(|p| p.key == "kill") {
                parameters.push(ActionParameter {
                    key: "kill".to_owned(),
                    value: json!("false"),
                });
            }
            InstantAction {
                action_type: action.action_type.clone(),
                action_id: action
                    .action_id
                    .clone()
                    .unwrap_or_else(|| format!("ia{}", index + 1)),
                action_descriptor: None,
                // Unlike an order action (default HARD: the robot finishes it
                // before driving on), an instant action runs *in parallel* with
                // movement, so the v3 schema allows only `NONE` here.
                blocking_type: Some(BlockingType::NONE),
                action_parameters: parameters,
            }
        })
        .collect();

    let message = InstantActions {
        header_id: next_header_id(),
        timestamp: crate::ingest::now_string(),
        version: vda5050_types::SPEC_VERSION.to_owned(),
        manufacturer: manufacturer.to_owned(),
        serial_number: serial_number.to_owned(),
        actions,
    };

    let topic = publisher
        .publish(
            Topic::InstantActions,
            &message.manufacturer,
            &message.serial_number,
            &message,
        )
        .await
        .map_err(|error| vec![error.to_string()])?;

    let applied: Vec<String> = message
        .actions
        .iter()
        .map(|a| a.action_type.clone())
        .collect();
    info!(
        %topic,
        %robot_id,
        header_id = message.header_id,
        actions = %applied.join(","),
        "instant actions published"
    );
    store
        .record(crate::store::GatewayEvent::InstantActionPublished {
            robot_id: robot_id.to_owned(),
            actions: applied,
            header_id: message.header_id,
            topic,
        })
        .await;
    Ok(message)
}

/// A `headerId` that is unique across gateway restarts.
///
/// Wall-clock based, like the order sequence, so a restarted gateway does not
/// reuse an id a robot has already answered.
fn next_header_id() -> u64 {
    u64::try_from(
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(1, |d| d.as_millis()),
    )
    .unwrap_or(1)
}
