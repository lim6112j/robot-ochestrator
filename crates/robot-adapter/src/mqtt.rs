//! An adapter that speaks VDA 5050 over MQTT.
//!
//! This is the Rust counterpart of `deploy/rmf/fleet_adapter`: it implements
//! [`RobotAdapter`] against the same wire protocol, so the two are
//! interchangeable from the gateway's point of view.
//!
//! It exists to prove the boundary is real. ADR-004 picked Python for the RMF
//! adapter because `rclrs` still declares "no stability guarantees"; when that
//! changes, a Rust adapter has to be a drop-in replacement rather than a
//! rewrite of the gateway. Having both implementations against one trait is
//! how that stays true.

use std::collections::BTreeMap;
use std::sync::Arc;

use async_trait::async_trait;
use tokio::sync::RwLock;
use vda5050_types::canonical::{CanonicalRobot, ConnectionState};
use vda5050_types::types::{InstantActions, Order, State};
use vda5050_types::{Topic, TopicPath};

use crate::{AdapterError, ResourceKind, RobotAdapter, SharedResource};

/// One robot as last seen on MQTT.
#[derive(Debug, Clone)]
struct Observed {
    state: Option<State>,
    /// `Offline` until a `state` or `connection` message says otherwise — the
    /// honest starting point for a robot nobody has heard from.
    connection: ConnectionState,
}

impl Default for Observed {
    fn default() -> Self {
        Self {
            state: None,
            connection: ConnectionState::Offline,
        }
    }
}

/// A fleet adapter backed by an MQTT connection.
///
/// Robot state is kept from `state` messages the ingest loop has already seen,
/// so this adapter and the dashboard cannot disagree about where a robot is.
#[derive(Debug)]
pub struct MqttRobotAdapter {
    manufacturer: String,
    interface: String,
    major_version: u8,
    /// Latest observation per `serialNumber`.
    observed: Arc<RwLock<BTreeMap<String, Observed>>>,
    /// Node ids that behave as doors, and therefore need mutual exclusion.
    door_nodes: Vec<String>,
}

impl MqttRobotAdapter {
    /// An adapter for one manufacturer.
    #[must_use]
    pub fn new(
        manufacturer: impl Into<String>,
        interface: impl Into<String>,
        major_version: u8,
    ) -> Self {
        Self {
            manufacturer: manufacturer.into(),
            interface: interface.into(),
            major_version,
            observed: Arc::new(RwLock::new(BTreeMap::new())),
            door_nodes: Vec::new(),
        }
    }

    /// Declare which node ids are shared resources.
    ///
    /// Without this the adapter has no way to know that two robots wanting
    /// `DOOR-IN` are contending for one physical doorway.
    #[must_use]
    pub fn with_door_nodes(mut self, nodes: Vec<String>) -> Self {
        self.door_nodes = nodes;
        self
    }

    /// Whether this adapter answers for a robot id.
    fn owns(&self, robot_id: &str) -> bool {
        robot_id
            .split_once(':')
            .is_some_and(|(manufacturer, _)| manufacturer == self.manufacturer)
    }

    /// The topic a message for this robot goes out on.
    fn topic(&self, serial_number: &str, topic: Topic) -> TopicPath {
        TopicPath::new(
            self.interface.clone(),
            self.major_version,
            self.manufacturer.clone(),
            serial_number.to_owned(),
            topic,
        )
    }
}

#[async_trait]
impl RobotAdapter for MqttRobotAdapter {
    fn manufacturer(&self) -> &str {
        &self.manufacturer
    }

    /// These two must live *inside* the trait impl: as inherent methods they
    /// are invisible through `Arc<dyn RobotAdapter>`, so the trait's no-op
    /// default would run and the adapter would never see a robot.
    async fn observe_state(&self, state: &State) {
        let mut observed = self.observed.write().await;
        let entry = observed.entry(state.serial_number.clone()).or_default();
        entry.connection = ConnectionState::Online;
        entry.state = Some(state.clone());
    }

    async fn observe_connection(&self, serial_number: &str, connection: ConnectionState) {
        let mut observed = self.observed.write().await;
        observed
            .entry(serial_number.to_owned())
            .or_default()
            .connection = connection;
    }

    async fn robots(&self) -> Vec<CanonicalRobot> {
        let observed = self.observed.read().await;
        observed
            .values()
            .filter_map(|entry| {
                let state = entry.state.as_ref()?;
                // A `state` message carries its own manufacturer, so trust it
                // over the key: an adapter that reports another fleet's robots
                // would make the coordination layer negotiate for a fleet it
                // does not speak for.
                if state.manufacturer != self.manufacturer {
                    return None;
                }
                Some(CanonicalRobot::from_state(
                    state,
                    entry.connection,
                    vda5050_types::SPEC_VERSION.to_owned(),
                ))
            })
            .collect()
    }

    async fn dispatch(&self, robot_id: &str, order: &Order) -> Result<(), AdapterError> {
        let Some((_, serial)) = robot_id.split_once(':') else {
            return Err(AdapterError::UnknownRobot {
                robot_id: robot_id.to_owned(),
            });
        };
        if !self.owns(robot_id) {
            return Err(AdapterError::UnknownRobot {
                robot_id: robot_id.to_owned(),
            });
        }
        let observed = self.observed.read().await;
        let seen = observed.get(serial).and_then(|o| o.state.as_ref());
        if seen.is_none() {
            // Never having seen a robot is not the same as it being idle: the
            // mission would go to a robot the adapter cannot place.
            return Err(AdapterError::UnknownRobot {
                robot_id: robot_id.to_owned(),
            });
        }
        // Publishing is the ingest loop's job in the running gateway; an
        // adapter that only observes cannot dispatch, and saying so is better
        // than silently accepting a mission that never leaves.
        let _ = self.topic(serial, Topic::Order);
        let _ = order;
        Err(AdapterError::Unsupported {
            manufacturer: self.manufacturer.clone(),
            capability: "dispatch (observe-only adapter)".to_owned(),
        })
    }

    async fn instant_action(
        &self,
        robot_id: &str,
        _actions: &InstantActions,
    ) -> Result<(), AdapterError> {
        if !self.owns(robot_id) {
            return Err(AdapterError::UnknownRobot {
                robot_id: robot_id.to_owned(),
            });
        }
        Err(AdapterError::Unsupported {
            manufacturer: self.manufacturer.clone(),
            capability: "instant_action (observe-only adapter)".to_owned(),
        })
    }

    async fn shared_resources(&self) -> Vec<SharedResource> {
        // Only doors that some known robot's route can reach are worth
        // reporting; an empty contender list means "any robot might want it".
        self.door_nodes
            .iter()
            .map(|id| SharedResource {
                id: id.clone(),
                kind: ResourceKind::Door,
                contenders: Vec::new(),
            })
            .collect()
    }

    async fn holds(&self, robot_id: &str, resource: &str) -> bool {
        if !self.door_nodes.iter().any(|d| d == resource) {
            return false;
        }
        let Some((_, serial)) = robot_id.split_once(':') else {
            return false;
        };
        let observed = self.observed.read().await;
        // A robot holds a door when it is stopped on one of the nodes that
        // touch it. Without route data this is the honest approximation.
        observed
            .get(serial)
            .and_then(|o| o.state.as_ref())
            .is_some_and(|state| {
                state
                    .node_states
                    .iter()
                    .any(|node| node.node_id == resource)
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal but valid `state`, built from JSON so the test does not have
    /// to track every field the wire type grows.
    fn state_on(serial: &str, node: &str) -> State {
        serde_json::from_value(serde_json::json!({
            "headerId": 1,
            "timestamp": "2026-01-01T00:00:00.000Z",
            "version": "3.0.0",
            "manufacturer": "acme",
            "serialNumber": serial,
            "orderId": "o1",
            "orderUpdateId": 0,
            "lastNodeId": node,
            "lastNodeSequenceId": 0,
            "nodeStates": [{
                "nodeId": node, "sequenceId": 0, "released": true, "actions": []
            }],
            "edgeStates": [],
            "driving": false,
            "actionStates": [],
            "instantActionStates": [],
            "powerSupply": { "stateOfCharge": 88.5, "charging": false },
            "operatingMode": "AUTOMATIC",
            "errors": [],
            "safetyState": { "activeEmergencyStop": "NONE", "fieldViolation": false }
        }))
        .expect("valid state")
    }

    fn adapter() -> MqttRobotAdapter {
        MqttRobotAdapter::new("acme", "vda5050", 3)
            .with_door_nodes(vec!["DOOR-IN".to_owned(), "DOOR-OUT".to_owned()])
    }

    #[tokio::test]
    async fn an_observed_robot_becomes_a_canonical_robot() {
        let adapter = adapter();
        assert!(adapter.robots().await.is_empty(), "nothing observed yet");
        adapter.observe_state(&state_on("AMR-001", "AS-1M")).await;
        let robots = adapter.robots().await;
        assert_eq!(robots.len(), 1);
        assert_eq!(robots[0].robot_id, "acme:AMR-001");
    }

    #[tokio::test]
    async fn an_adapter_ignores_another_manufacturers_state() {
        // The key is the serial number, but the message carries its own
        // manufacturer. Without this check an adapter would report a robot it
        // does not speak for, and the coordination layer would negotiate on
        // its behalf.
        let adapter = adapter();
        adapter.observe_state(&state_on("AMR-001", "AS-1M")).await;
        // Hand-built state that claims a different manufacturer.
        let mut foreign = state_on("SCR-001", "AS-1M");
        foreign.manufacturer = "sweepco".to_owned();
        adapter.observe_state(&foreign).await;

        let robots = adapter.robots().await;
        assert_eq!(robots.len(), 1, "only this adapter's own fleet");
        assert_eq!(robots[0].robot_id, "acme:AMR-001");
    }

    #[tokio::test]
    async fn a_robot_it_has_never_seen_cannot_be_dispatched_to() {
        // Silently accepting would look like a slow robot; refusing makes the
        // operator's screen show the truth.
        let adapter = adapter();
        let order = Order {
            header_id: 1,
            timestamp: "2026-01-01T00:00:00.000Z".to_owned(),
            version: "3.0.0".to_owned(),
            manufacturer: "acme".to_owned(),
            serial_number: "AMR-001".to_owned(),
            order_id: "o1".to_owned(),
            order_update_id: 0,
            order_description: None,
            nodes: Vec::new(),
            edges: Vec::new(),
        };
        let err = adapter
            .dispatch("acme:AMR-001", &order)
            .await
            .expect_err("never seen");
        assert!(matches!(err, AdapterError::UnknownRobot { .. }));
    }

    #[tokio::test]
    async fn another_manufacturers_robot_is_not_ours() {
        let adapter = adapter();
        let order = Order {
            header_id: 1,
            timestamp: "2026-01-01T00:00:00.000Z".to_owned(),
            version: "3.0.0".to_owned(),
            manufacturer: "sweepco".to_owned(),
            serial_number: "SCR-001".to_owned(),
            order_id: "o1".to_owned(),
            order_update_id: 0,
            order_description: None,
            nodes: Vec::new(),
            edges: Vec::new(),
        };
        let err = adapter
            .dispatch("sweepco:SCR-001", &order)
            .await
            .expect_err("not this adapter's fleet");
        assert!(matches!(err, AdapterError::UnknownRobot { .. }));
    }

    #[tokio::test]
    async fn declared_doors_are_reported_as_shared_resources() {
        let adapter = adapter();
        let resources = adapter.shared_resources().await;
        assert_eq!(resources.len(), 2);
        assert!(resources.iter().all(|r| r.kind == ResourceKind::Door));
        assert!(resources.iter().any(|r| r.id == "DOOR-IN"));
    }

    #[tokio::test]
    async fn a_robot_stopped_on_a_door_node_holds_it() {
        let adapter = adapter();
        adapter.observe_state(&state_on("AMR-001", "DOOR-IN")).await;
        assert!(adapter.holds("acme:AMR-001", "DOOR-IN").await);
        // A node that is not a declared door is never held.
        adapter.observe_state(&state_on("AMR-002", "AS-1M")).await;
        assert!(!adapter.holds("acme:AMR-002", "AS-1M").await);
    }

    #[tokio::test]
    async fn a_lost_connection_marks_the_robot_offline() {
        // The Last Will path: without this a dead robot keeps looking online.
        let adapter = adapter();
        adapter.observe_state(&state_on("AMR-001", "AS-1M")).await;
        assert_eq!(
            adapter.robots().await[0].connection,
            ConnectionState::Online
        );
        adapter
            .observe_connection("AMR-001", ConnectionState::ConnectionBroken)
            .await;
        assert_eq!(
            adapter.robots().await[0].connection,
            ConnectionState::ConnectionBroken
        );
    }

    #[test]
    fn the_topic_uses_the_v_prefixed_major_version() {
        let adapter = MqttRobotAdapter::new("acme", "vda5050", 3);
        assert_eq!(
            adapter.topic("AMR-001", Topic::Order).to_string(),
            "vda5050/v3/acme/AMR-001/order"
        );
    }
}
