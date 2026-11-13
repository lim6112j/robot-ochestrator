//! In-memory state of every known robot, plus DLQ, metrics and event log.

use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

use tokio::sync::{broadcast, RwLock};
use vda5050_types::canonical::{CanonicalRobot, ConnectionState};
use vda5050_types::Topic;

/// A message that failed schema validation.
#[derive(Debug, Clone)]
pub struct DlqEntry {
    pub topic: String,
    pub topic_kind: Topic,
    pub reasons: Vec<String>,
}

/// An operational event worth showing in the UI.
#[derive(Debug, Clone, serde::Serialize)]
#[serde(tag = "type", rename_all = "camelCase")]
pub enum GatewayEvent {
    RobotOnline {
        robot_id: String,
    },
    RobotOffline {
        robot_id: String,
        reason: String,
    },
    OrderAccepted {
        robot_id: String,
        order_id: String,
    },
    OrderRejected {
        robot_id: String,
        order_id: String,
        reason: String,
    },
    SchemaViolation {
        topic: String,
        reasons: Vec<String>,
    },
    OrderPublished {
        robot_id: String,
        order_id: String,
        header_id: u64,
        waypoints: usize,
        topic: String,
    },
    InstantActionPublished {
        robot_id: String,
        actions: Vec<String>,
        header_id: u64,
        topic: String,
    },
}

/// Counters exposed on `/api/metrics`.
#[derive(Debug, Default, Clone, serde::Serialize)]
pub struct Metrics {
    pub messages_seen: u64,
    pub messages_valid: u64,
    pub messages_rejected: u64,
    pub orders_seen: u64,
    /// Missions the dashboard asked the gateway to publish.
    pub orders_published: u64,
    pub robots_online: u64,
    pub robots_offline: u64,
}

/// The shared gateway state.
///
/// `Debug` is hand-written: the field is a map of trait objects, which cannot
/// derive it, and a derived `Debug` on the whole store is more use than a
/// manual one for every field.
pub struct Store {
    /// Registered fleet adapters, keyed by manufacturer.
    ///
    /// The gateway does not drive robots through these — it dispatches over
    /// MQTT — but they answer "what can this fleet do", which is what the
    /// coordination layer needs and what makes an adapter swappable.
    adapters: RwLock<HashMap<String, Arc<dyn robot_adapter::RobotAdapter>>>,
    robots: RwLock<HashMap<String, CanonicalRobot>>,
    events: RwLock<VecDeque<GatewayEvent>>,
    dlq: RwLock<VecDeque<DlqEntry>>,
    metrics: RwLock<Metrics>,
    tx: broadcast::Sender<String>,
    /// Robots not seen for this long are considered offline.
    heartbeat_timeout_s: f64,
    max_log: usize,
}

/// Maximum number of kept events and DLQ entries.
const DEFAULT_MAX_LOG: usize = 500;

impl std::fmt::Debug for Store {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Store")
            .field("robots", &self.robots.try_read().map(|r| r.len()))
            .field("events", &self.events.try_read().map(|e| e.len()))
            .field("dlq", &self.dlq.try_read().map(|d| d.len()))
            .field("adapters", &self.adapters.try_read().map(|a| a.len()))
            .field("metrics", &self.metrics.try_read().map(|m| m.messages_seen))
            .finish_non_exhaustive()
    }
}

impl Store {
    /// Create a new store.
    #[must_use]
    pub fn new(heartbeat_timeout_s: f64) -> Arc<Self> {
        let (tx, _rx) = broadcast::channel(256);
        Arc::new(Self {
            adapters: RwLock::new(HashMap::new()),
            robots: RwLock::new(HashMap::new()),
            events: RwLock::new(VecDeque::new()),
            dlq: RwLock::new(VecDeque::new()),
            metrics: RwLock::new(Metrics::default()),
            tx,
            heartbeat_timeout_s,
            max_log: DEFAULT_MAX_LOG,
        })
    }

    /// Subscribe to the JSON event stream used by the WebSocket endpoint.
    #[must_use]
    pub fn subscribe(&self) -> broadcast::Receiver<String> {
        self.tx.subscribe()
    }

    /// Register a fleet adapter for one manufacturer.
    pub async fn register_adapter(&self, adapter: Arc<dyn robot_adapter::RobotAdapter>) {
        self.adapters
            .write()
            .await
            .insert(adapter.manufacturer().to_owned(), adapter);
    }

    /// Every registered adapter.
    pub async fn adapters(&self) -> Vec<Arc<dyn robot_adapter::RobotAdapter>> {
        self.adapters.read().await.values().cloned().collect()
    }

    /// The adapter for a robot id, if one is registered for its manufacturer.
    pub async fn adapter_for(
        &self,
        robot_id: &str,
    ) -> Option<Arc<dyn robot_adapter::RobotAdapter>> {
        let manufacturer = robot_id.split_once(':').map(|(m, _)| m)?;
        self.adapters.read().await.get(manufacturer).cloned()
    }

    /// Shared resources across every registered adapter.
    pub async fn shared_resources(&self) -> Vec<robot_adapter::SharedResource> {
        let adapters = self.adapters.read().await;
        let mut resources = Vec::new();
        for adapter in adapters.values() {
            resources.extend(adapter.shared_resources().await);
        }
        resources
    }

    /// Feed a `state` message to the adapters that speak for its manufacturer.
    ///
    /// The adapters are not in the dispatch path — the gateway publishes over
    /// MQTT — but they answer "what can this fleet do", which is only correct
    /// if they see the same states the gateway does.
    pub async fn observe(&self, state: &vda5050_types::types::State) {
        let adapters = self.adapters.read().await;
        if let Some(adapter) = adapters.get(&state.manufacturer) {
            let adapter = Arc::clone(adapter);
            drop(adapters);
            adapter.observe_state(state).await;
        }
    }

    /// Feed a connection change to the adapters, including Last Will teardowns.
    pub async fn observe_connection(
        &self,
        robot_id: &str,
        connection: vda5050_types::canonical::ConnectionState,
    ) {
        let Some((manufacturer, serial)) = robot_id.split_once(':') else {
            return;
        };
        let adapters = self.adapters.read().await;
        if let Some(adapter) = adapters.get(manufacturer) {
            let adapter = Arc::clone(adapter);
            drop(adapters);
            adapter.observe_connection(serial, connection).await;
        }
    }

    /// Insert or update a robot.
    pub async fn upsert(&self, robot: CanonicalRobot) {
        let previous = self.robots.read().await.get(&robot.robot_id).cloned();
        let became_online = robot.connection == ConnectionState::Online
            && previous.as_ref().map(|p| p.connection) != Some(ConnectionState::Online);
        let became_offline = robot.connection != ConnectionState::Online
            && previous.as_ref().map(|p| p.connection) == Some(ConnectionState::Online);

        // Push the new state to every connected dashboard: without this the
        // digital twin would only ever show the initial snapshot.
        if let Ok(json) = serde_json::to_string(&serde_json::json!({
            "type": "robot",
            "robot": robot,
        })) {
            let _ = self.tx.send(json);
        }

        self.robots
            .write()
            .await
            .insert(robot.robot_id.clone(), robot);

        if became_online {
            self.record(GatewayEvent::RobotOnline {
                robot_id: previous
                    .as_ref()
                    .map_or_else(String::new, |p| p.robot_id.clone()),
            })
            .await;
        }
        if became_offline {
            self.record(GatewayEvent::RobotOffline {
                robot_id: previous
                    .as_ref()
                    .map_or_else(String::new, |p| p.robot_id.clone()),
                reason: "connection".to_owned(),
            })
            .await;
        }
    }

    /// Apply a patch to a known robot (used by `connection` messages).
    pub async fn set_connection(
        &self,
        robot_id: &str,
        state: ConnectionState,
        now: String,
    ) -> bool {
        let mut robots = self.robots.write().await;
        let Some(robot) = robots.get_mut(robot_id) else {
            return false;
        };
        let changed = robot.connection != state;
        robot.connection = state;
        robot.last_seen.clone_from(&now);
        drop(robots);
        if changed {
            self.observe_connection(robot_id, state).await;
            let event = match state {
                ConnectionState::Online => GatewayEvent::RobotOnline {
                    robot_id: robot_id.to_owned(),
                },
                _ => GatewayEvent::RobotOffline {
                    robot_id: robot_id.to_owned(),
                    reason: state.as_str().to_owned(),
                },
            };
            self.record(event).await;
        }
        true
    }

    /// Mark robots whose heartbeat expired as offline.
    pub async fn expire_stale(&self, now: &str) {
        let mut robots = self.robots.write().await;
        let mut changed = Vec::new();
        for robot in robots.values_mut() {
            if robot.connection == ConnectionState::Online
                && is_stale(&robot.last_seen, now, self.heartbeat_timeout_s)
            {
                robot.connection = ConnectionState::ConnectionBroken;
                changed.push(robot.robot_id.clone());
            }
        }
        drop(robots);
        for robot_id in changed {
            self.observe_connection(&robot_id, ConnectionState::ConnectionBroken)
                .await;
            self.record(GatewayEvent::RobotOffline {
                robot_id,
                reason: "heartbeat timeout".to_owned(),
            })
            .await;
        }
    }

    /// All robots, sorted by id.
    pub async fn robots(&self) -> Vec<CanonicalRobot> {
        let mut robots: Vec<CanonicalRobot> = self.robots.read().await.values().cloned().collect();
        robots.sort_by(|a, b| a.robot_id.cmp(&b.robot_id));
        robots
    }

    /// One robot by id.
    pub async fn robot(&self, robot_id: &str) -> Option<CanonicalRobot> {
        self.robots.read().await.get(robot_id).cloned()
    }

    /// Record an event and broadcast it to WebSocket clients.
    pub async fn record(&self, event: GatewayEvent) {
        {
            let mut log = self.events.write().await;
            log.push_back(event.clone());
            while log.len() > self.max_log {
                log.pop_front();
            }
        }
        if let Ok(json) = serde_json::to_string(&event) {
            let _ = self.tx.send(json);
        }
    }

    /// Recent events, newest last.
    pub async fn events(&self, limit: usize) -> Vec<GatewayEvent> {
        let log = self.events.read().await;
        log.iter().rev().take(limit).rev().cloned().collect()
    }

    /// Record a schema violation in the DLQ.
    pub async fn record_rejection(&self, entry: DlqEntry) {
        {
            let mut dlq = self.dlq.write().await;
            dlq.push_back(entry);
            while dlq.len() > self.max_log {
                dlq.pop_front();
            }
        }
        self.metrics.write().await.messages_rejected += 1;
    }

    /// Recent DLQ entries.
    pub async fn dlq(&self, limit: usize) -> Vec<DlqEntry> {
        let dlq = self.dlq.read().await;
        dlq.iter().rev().take(limit).rev().cloned().collect()
    }

    /// Bump the message counters.
    pub async fn count_message(&self, valid: bool, topic: Topic) {
        let mut metrics = self.metrics.write().await;
        metrics.messages_seen += 1;
        if valid {
            metrics.messages_valid += 1;
        }
        if topic == Topic::Order {
            metrics.orders_seen += 1;
        }
    }

    /// Count a mission the gateway published for the dashboard.
    pub async fn count_published(&self) {
        self.metrics.write().await.orders_published += 1;
    }

    /// Snapshot of the counters.
    pub async fn metrics(&self) -> Metrics {
        let metrics = self.metrics.read().await.clone();
        let online = self
            .robots
            .read()
            .await
            .values()
            .filter(|r| r.connection == ConnectionState::Online)
            .count() as u64;
        Metrics {
            robots_online: online,
            ..metrics
        }
    }
}

/// True when `last_seen` is older than `timeout_s`.
fn is_stale(last_seen: &str, now: &str, timeout_s: f64) -> bool {
    match (parse_ts(last_seen), parse_ts(now)) {
        (Some(a), Some(b)) => (b - a) > timeout_s,
        _ => false,
    }
}

/// Parse the ISO-8601 timestamps used by VDA 5050 into epoch seconds.
#[must_use]
pub fn parse_ts(raw: &str) -> Option<f64> {
    // 2026-01-01T00:00:00.123Z
    let date = raw.get(0..10)?;
    let time = raw.get(11..19)?;
    let millis = raw
        .get(20..23)
        .and_then(|m| m.parse::<f64>().ok())
        .unwrap_or(0.0);
    let year: i64 = date.get(0..4)?.parse().ok()?;
    let month: i64 = date.get(5..7)?.parse().ok()?;
    let day: i64 = date.get(8..10)?.parse().ok()?;
    let hour: i64 = time.get(0..2)?.parse().ok()?;
    let minute: i64 = time.get(3..5)?.parse().ok()?;
    let second: i64 = time.get(6..8)?.parse().ok()?;
    #[allow(
        clippy::cast_precision_loss,
        reason = "epoch seconds fit in f64 for the date range used here"
    )]
    let epoch = days_from_civil(year, month, day) as f64 * 86_400.0
        + (hour * 3_600 + minute * 60 + second) as f64
        + millis / 1_000.0;
    Some(epoch)
}

/// Days since 1970-01-01 (Howard Hinnant's algorithm).
fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_vda5050_timestamps() {
        let a = parse_ts("2026-01-01T00:00:00.000Z").expect("parses");
        let b = parse_ts("2026-01-01T00:00:01.500Z").expect("parses");
        assert!((b - a - 1.5).abs() < 1e-6);
        assert!(parse_ts("not-a-timestamp").is_none());
    }

    #[tokio::test]
    async fn upsert_tracks_online_transition() {
        let store = Store::new(5.0);
        let robot = CanonicalRobot {
            robot_id: "acme:AMR-001".to_owned(),
            manufacturer: "acme".to_owned(),
            serial_number: "AMR-001".to_owned(),
            fleet: vda5050_types::canonical::FleetKind::Logistics,
            source: vda5050_types::canonical::Source::Simulator,
            interface_name: "vda5050".to_owned(),
            major_version: 3,
            spec_version: "3.0.0".to_owned(),
            map_id: Some("plant-a/l1".to_owned()),
            position: None,
            power: None,
            operating_mode: "AUTOMATIC".to_owned(),
            driving: false,
            paused: false,
            order_id: String::new(),
            action_states: Vec::new(),
            instant_action_states: Vec::new(),
            errors: Vec::new(),
            safety: None,
            connection: ConnectionState::Online,
            last_seen: "2026-01-01T00:00:00.000Z".to_owned(),
            capabilities: vda5050_types::canonical::Capabilities::default(),
        };
        store.upsert(robot).await;
        assert_eq!(store.robots().await.len(), 1);
        let events = store.events(10).await;
        assert!(matches!(
            events.first(),
            Some(GatewayEvent::RobotOnline { .. })
        ));
    }

    #[tokio::test]
    async fn stale_robots_go_offline() {
        let store = Store::new(1.0);
        store
            .upsert(CanonicalRobot {
                last_seen: "2026-01-01T00:00:00.000Z".to_owned(),
                connection: ConnectionState::Online,
                ..test_robot()
            })
            .await;
        store.expire_stale("2026-01-01T00:00:10.000Z").await;
        let robot = store.robot("acme:AMR-001").await.expect("robot exists");
        assert_eq!(robot.connection, ConnectionState::ConnectionBroken);
    }

    fn test_robot() -> CanonicalRobot {
        CanonicalRobot {
            robot_id: "acme:AMR-001".to_owned(),
            manufacturer: "acme".to_owned(),
            serial_number: "AMR-001".to_owned(),
            fleet: vda5050_types::canonical::FleetKind::Logistics,
            source: vda5050_types::canonical::Source::Simulator,
            interface_name: "vda5050".to_owned(),
            major_version: 3,
            spec_version: "3.0.0".to_owned(),
            map_id: None,
            position: None,
            power: None,
            operating_mode: "AUTOMATIC".to_owned(),
            driving: false,
            paused: false,
            order_id: String::new(),
            action_states: Vec::new(),
            instant_action_states: Vec::new(),
            errors: Vec::new(),
            safety: None,
            connection: ConnectionState::Online,
            last_seen: "2026-01-01T00:00:00.000Z".to_owned(),
            capabilities: vda5050_types::canonical::Capabilities::default(),
        }
    }
}
