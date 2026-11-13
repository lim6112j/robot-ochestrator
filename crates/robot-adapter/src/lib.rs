//! The contract between the gateway and any fleet adapter.
//!
//! An adapter is whatever turns a robot — or a fleet manager fronting several
//! robots — into the four things the gateway needs: observed state, and the
//! ability to dispatch a mission, issue an instant action, and negotiate for a
//! shared resource such as a door.
//!
//! ## Why this exists
//!
//! ADR-004 chose Python `fleet_adapter_template` for the RMF boundary, which
//! knowingly breaks the Rust-only rule for that one component. The reason that
//! is acceptable rather than a hole in the design is *this trait*: the adapter
//! is a replaceable part, not load-bearing architecture. When `rclrs` declares
//! stability guarantees, a Rust adapter implements the same four methods and
//! the gateway does not change.
//!
//! The trait is deliberately small. Every method here is something the current
//! Python adapter already does over MQTT; nothing is speculative.
//!
//! ## What is *not* here
//!
//! Traffic negotiation itself. Deciding which robot goes first through a
//! narrow aisle is RMF's job, and it reaches the adapter through the shared
//! resource protocol below — the adapter only reports what it can do and
//! carries out what it is told.

pub mod mqtt;

use async_trait::async_trait;
use vda5050_types::canonical::CanonicalRobot;
use vda5050_types::types::{InstantActions, Order};

/// What an adapter can do for one robot.
///
/// Implementations are per-fleet, not per-robot: one adapter speaks for every
/// robot of a manufacturer, exactly as VDA 5050 addresses them.
#[async_trait]
pub trait RobotAdapter: Send + Sync {
    /// The fleet this adapter speaks for, e.g. `acme`.
    fn manufacturer(&self) -> &str;

    /// Record a `state` message the gateway has already validated.
    ///
    /// The default is a no-op, because most adapters do not need to see state
    /// to answer the trait. [`robot_adapter::mqtt::MqttRobotAdapter`] overrides
    /// it, since it reports robots it has observed.
    async fn observe_state(&self, _state: &vda5050_types::types::State) {}

    /// Record a connection change, including a Last Will teardown.
    async fn observe_connection(
        &self,
        _serial_number: &str,
        _connection: vda5050_types::canonical::ConnectionState,
    ) {
    }

    /// Every robot this adapter currently knows about.
    ///
    /// The gateway merges these with what it observes on MQTT, so an adapter
    /// that returns an empty list is not broken — it simply has not seen any
    /// robot yet.
    async fn robots(&self) -> Vec<CanonicalRobot>;

    /// Dispatch a mission. The order is already schema-valid.
    ///
    /// # Errors
    /// Returns [`AdapterError::Dispatch`] when the robot or its fleet manager
    /// refused the mission, and [`AdapterError::Transport`] when it could not
    /// be reached at all. The two are different operationally: the first means
    /// the mission was rejected and nothing is running, the second means the
    /// operator cannot tell.
    async fn dispatch(&self, robot_id: &str, order: &Order) -> Result<(), AdapterError>;

    /// Issue instant actions (pause, resume, cancel, …).
    ///
    /// # Errors
    /// Same contract as [`RobotAdapter::dispatch`].
    async fn instant_action(
        &self,
        robot_id: &str,
        actions: &InstantActions,
    ) -> Result<(), AdapterError>;

    /// Shared resources this adapter's robots contend for.
    ///
    /// A door is the canonical example: two robots from different fleets
    /// wanting the same doorway must not both be told to go. Returning a
    /// resource here is what lets the coordination layer reserve it and hold
    /// the second robot back.
    async fn shared_resources(&self) -> Vec<SharedResource>;

    /// Whether a robot currently holds a shared resource.
    ///
    /// The gateway uses this to explain *why* a robot is waiting, which is
    /// otherwise invisible: a robot stopped at a door and a robot that lost
    /// its connection look identical in a position plot.
    async fn holds(&self, robot_id: &str, resource: &str) -> bool;
}

/// A resource several robots may want but only one may use at a time.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct SharedResource {
    /// Identifier the coordination layer reserves, e.g. a node id.
    pub id: String,
    /// What kind of thing it is, so a reader can tell a door from a lift.
    pub kind: ResourceKind,
    /// Robots whose routes pass through it, if known.
    ///
    /// Empty means "any robot might want this", which is the honest answer
    /// before a route is planned.
    pub contenders: Vec<String>,
}

/// The kinds of shared resource a warehouse actually has.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub enum ResourceKind {
    /// A doorway. Narrow, and the common case.
    Door,
    /// An elevator or lift.
    Lift,
    /// Anything else that needs mutual exclusion.
    Mutex,
}

/// Why an adapter could not carry out a request.
#[derive(Debug, thiserror::Error)]
pub enum AdapterError {
    #[error("adapter does not know robot '{robot_id}'")]
    UnknownRobot { robot_id: String },

    #[error("{robot_id} refused the request: {reason}")]
    Dispatch { robot_id: String, reason: String },

    #[error("could not reach {robot_id}: {reason}")]
    Transport { robot_id: String, reason: String },

    #[error("adapter for '{manufacturer}' does not support {capability}")]
    Unsupported {
        manufacturer: String,
        capability: String,
    },
}

/// An adapter that answers for nobody.
///
/// Useful as a default while a fleet is still being onboarded: the gateway
/// keeps working, reports the fleet as having no robots, and nothing pretends
/// to be able to drive hardware it cannot reach.
#[derive(Debug, Default, Clone, Copy)]
pub struct NullAdapter {
    manufacturer: &'static str,
}

impl NullAdapter {
    /// An adapter for a manufacturer with nothing behind it yet.
    #[must_use]
    pub const fn new(manufacturer: &'static str) -> Self {
        Self { manufacturer }
    }
}

#[async_trait]
impl RobotAdapter for NullAdapter {
    fn manufacturer(&self) -> &str {
        self.manufacturer
    }

    async fn robots(&self) -> Vec<CanonicalRobot> {
        Vec::new()
    }

    async fn dispatch(&self, robot_id: &str, _order: &Order) -> Result<(), AdapterError> {
        Err(AdapterError::UnknownRobot {
            robot_id: robot_id.to_owned(),
        })
    }

    async fn instant_action(
        &self,
        robot_id: &str,
        _actions: &InstantActions,
    ) -> Result<(), AdapterError> {
        Err(AdapterError::UnknownRobot {
            robot_id: robot_id.to_owned(),
        })
    }

    async fn shared_resources(&self) -> Vec<SharedResource> {
        Vec::new()
    }

    async fn holds(&self, _robot_id: &str, _resource: &str) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn a_null_adapter_knows_nobody() {
        let adapter = NullAdapter::new("acme");
        assert_eq!(adapter.manufacturer(), "acme");
        assert!(adapter.robots().await.is_empty());
        assert!(adapter.shared_resources().await.is_empty());
        assert!(!adapter.holds("acme:AMR-001", "DOOR-IN").await);
    }

    #[tokio::test]
    async fn a_null_adapter_refuses_rather_than_pretending() {
        // Dispatching through an adapter with nothing behind it must fail
        // loudly: silently doing nothing would look like a slow robot.
        let adapter = NullAdapter::new("acme");
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
            .expect_err("no robot behind a null adapter");
        assert!(matches!(err, AdapterError::UnknownRobot { .. }));
        assert!(err.to_string().contains("acme:AMR-001"));
    }

    #[test]
    fn a_shared_resource_names_what_robots_contend_for() {
        let door = SharedResource {
            id: "DOOR-IN".to_owned(),
            kind: ResourceKind::Door,
            contenders: vec!["acme:AMR-001".to_owned(), "sweepco:SCR-001".to_owned()],
        };
        assert_eq!(door.kind, ResourceKind::Door);
        assert_eq!(door.contenders.len(), 2);
        // Equality is by value, so a reservation table can use these as keys.
        assert_eq!(door.clone(), door);
    }

    #[test]
    fn transport_and_refusal_are_distinguishable() {
        // An operator seeing "refused" knows nothing is running; seeing
        // "could not reach" does not. Collapsing them loses that.
        let refused = AdapterError::Dispatch {
            robot_id: "acme:AMR-001".to_owned(),
            reason: "battery too low".to_owned(),
        };
        let unreachable = AdapterError::Transport {
            robot_id: "acme:AMR-001".to_owned(),
            reason: "broker refused the publish".to_owned(),
        };
        assert!(matches!(refused, AdapterError::Dispatch { .. }));
        assert!(matches!(unreachable, AdapterError::Transport { .. }));
        assert_ne!(
            std::mem::discriminant(&refused),
            std::mem::discriminant(&unreachable)
        );
    }
}
