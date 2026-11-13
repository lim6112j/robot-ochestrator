//! MQTT ingest: subscribe, validate against the official schemas, normalise.

use std::sync::Arc;

use tokio::time::{interval, Duration};
use tracing::{debug, info, warn};
use vda5050_types::canonical::{ConnectionState, FleetKind};
use vda5050_types::types::{Connection, Factsheet, State};
use vda5050_types::v2::StateV2;
use vda5050_types::{MessageValidator, Topic, TopicPath};

use crate::store::{DlqEntry, GatewayEvent, Store};

/// Errors of the MQTT ingest loop.
#[derive(Debug, thiserror::Error)]
pub enum IngestError {
    #[error("mqtt client error: {0}")]
    Mqtt(#[from] rumqttc::ClientError),
    #[error("schema error: {0}")]
    Schema(#[from] vda5050_types::Error),
}

/// Fleet category inferred from the actions a robot supports.
fn infer_fleet(actions: &std::collections::BTreeSet<String>) -> FleetKind {
    let list: Vec<String> = actions.iter().cloned().collect();
    FleetKind::from_actions(&list)
}

/// Ingest configuration.
#[derive(Debug, Clone)]
pub struct IngestConfig {
    pub interface: String,
    pub major_version: u8,
    /// Also subscribe to v2 fleets on the legacy `uomg` interface.
    pub accept_v2: bool,
    pub heartbeat_timeout_s: f64,
}

/// Opens the broker connection the gateway uses for both directions.
///
/// The returned client is cheap to clone; the ingest loop drives its event
/// loop, which is also what flushes orders published by `/api/orders`.
pub fn connect(
    broker_host: &str,
    broker_port: u16,
    client_id: &str,
) -> (rumqttc::AsyncClient, rumqttc::EventLoop) {
    let mut options = rumqttc::MqttOptions::new(client_id, broker_host, broker_port);
    options.set_keep_alive(Duration::from_secs(10));
    rumqttc::AsyncClient::new(options, 512)
}

/// Runs the MQTT event loop until the process ends.
///
/// Takes the shared client of [`connect`] — it subscribes with it — plus
/// ownership of its event loop, which is what flushes orders published from
/// `/api/orders` as well.
pub async fn run(
    client: rumqttc::AsyncClient,
    mut eventloop: rumqttc::EventLoop,
    config: IngestConfig,
    store: Arc<Store>,
) -> Result<(), IngestError> {
    // Subscribe to every robot on the configured interface, plus the legacy v2 one.
    let mut subscriptions = Vec::new();
    for topic in [
        Topic::State,
        Topic::Visualization,
        Topic::Connection,
        Topic::Factsheet,
    ] {
        subscriptions.push(TopicPath::wildcard(
            &config.interface,
            config.major_version,
            topic,
        ));
    }
    if config.accept_v2 {
        subscriptions.push(TopicPath::wildcard("uomg", 2, Topic::State));
        subscriptions.push(TopicPath::wildcard("uomg", 2, Topic::Factsheet));
        subscriptions.push(TopicPath::wildcard("uomg", 2, Topic::Connection));
    }

    for subscription in subscriptions {
        client
            .subscribe(subscription.clone(), rumqttc::QoS::AtMostOnce)
            .await?;
        info!(%subscription, "subscribed");
    }

    let validator = MessageValidator::new()?;
    let mut ticker = interval(Duration::from_secs_f64(config.heartbeat_timeout_s / 2.0));

    loop {
        tokio::select! {
            event = eventloop.poll() => {
                match event {
                    Ok(rumqttc::Event::Incoming(rumqttc::Incoming::Publish(publish))) => {
                        let topic = publish.topic.clone();
                        let payload = publish.payload.to_vec();
                        handle_message(&validator, store.clone(), &topic, &payload).await;
                    }
                    Ok(_) => {}
                    Err(e) => warn!(error = %e, "mqtt connection error"),
                }
            }
            _ = ticker.tick() => {
                store.expire_stale(&now_string()).await;
            }
        }
    }
}

#[allow(
    clippy::too_many_lines,
    reason = "one branch per VDA 5050 topic keeps the routing table readable"
)]
async fn handle_message(
    validator: &MessageValidator,
    store: Arc<Store>,
    topic: &str,
    payload: &[u8],
) {
    let Ok(path) = TopicPath::parse(topic) else {
        debug!(%topic, "ignoring message on unparsable topic");
        return;
    };

    store.count_message(false, path.topic).await;

    match path.topic {
        Topic::State => {
            if path.major_version >= 3 {
                match validator.validate_bytes(Topic::State, payload) {
                    Ok(value) => match serde_json::from_value::<State>(value) {
                        Ok(state) => {
                            let mut robot = vda5050_types::CanonicalRobot::from_state(
                                &state,
                                ConnectionState::Online,
                                now_string(),
                            );
                            // A factSheet may have arrived before the first state: keep
                            // the discovered capabilities instead of overwriting them.
                            if let Some(existing) = store.robot(&robot.robot_id).await {
                                robot.capabilities = existing.capabilities;
                                robot.fleet = existing.fleet;
                                robot.source = existing.source;
                            }
                            store.count_message(true, Topic::State).await;
                            let first_seen = store.robot(&robot.robot_id).await.is_none();
                            let previous_order = store
                                .robot(&robot.robot_id)
                                .await
                                .map(|r| r.order_id)
                                .unwrap_or_default();
                            store.observe(&state).await;
                            store.upsert(robot.clone()).await;
                            if first_seen {
                                store
                                    .record(GatewayEvent::RobotOnline {
                                        robot_id: robot.robot_id.clone(),
                                    })
                                    .await;
                            }
                            // v3 has no `responses` handshake: a new orderId in the state
                            // means the robot accepted the order.
                            if !state.order_id.is_empty() && state.order_id != previous_order {
                                store
                                    .record(GatewayEvent::OrderAccepted {
                                        robot_id: robot.robot_id.clone(),
                                        order_id: state.order_id.clone(),
                                    })
                                    .await;
                            }
                            for error in &state.errors {
                                if error.error_type == "VALIDATION_FAILURE"
                                    || error.error_type == "INVALID_ORDER_ACTION"
                                {
                                    store
                                        .record(GatewayEvent::OrderRejected {
                                            robot_id: robot.robot_id.clone(),
                                            order_id: state.order_id.clone(),
                                            reason: error.error_type.clone(),
                                        })
                                        .await;
                                }
                            }
                        }
                        Err(e) => reject(store.clone(), &path, vec![e.to_string()]).await,
                    },
                    Err(rejected) => {
                        reject(store.clone(), &path, rejected.reasons).await;
                    }
                }
            } else {
                // v2 ingress: parse, convert to canonical.
                match serde_json::from_slice::<StateV2>(payload) {
                    Ok(state) => {
                        let mut robot = state.to_canonical(ConnectionState::Online, now_string());
                        if let Some(existing) = store.robot(&robot.robot_id).await {
                            robot.capabilities = existing.capabilities;
                            robot.fleet = existing.fleet;
                        }
                        store.count_message(true, Topic::State).await;
                        store.upsert(robot).await;
                    }
                    Err(e) => reject(store.clone(), &path, vec![e.to_string()]).await,
                }
            }
        }
        Topic::Connection => match validator.validate_bytes(Topic::Connection, payload) {
            Ok(value) => match serde_json::from_value::<Connection>(value) {
                Ok(connection) => {
                    store.count_message(true, Topic::Connection).await;
                    let robot_id =
                        format!("{}:{}", connection.manufacturer, connection.serial_number);
                    store
                        .set_connection(&robot_id, connection.connection_state.into(), now_string())
                        .await;
                }
                Err(e) => reject(store.clone(), &path, vec![e.to_string()]).await,
            },
            Err(rejected) => reject(store.clone(), &path, rejected.reasons).await,
        },
        Topic::Factsheet => {
            match validator.validate_bytes(Topic::Factsheet, payload) {
                Ok(value) => match serde_json::from_value::<Factsheet>(value) {
                    Ok(factsheet) => {
                        store.count_message(true, Topic::Factsheet).await;
                        let robot_id =
                            format!("{}:{}", factsheet.manufacturer, factsheet.serial_number);
                        let mut robots = store.robots().await;
                        if let Some(robot) = robots.iter_mut().find(|r| r.robot_id == robot_id) {
                            robot.apply_factsheet(&factsheet);
                            robot.fleet = infer_fleet(&robot.capabilities.actions);
                            store.upsert(robot.clone()).await;
                        } else {
                            // factSheet before the first state: create a placeholder entry.
                            let mut robot = placeholder(
                                &factsheet.manufacturer,
                                &factsheet.serial_number,
                                &factsheet.version,
                            );
                            robot.apply_factsheet(&factsheet);
                            robot.fleet = infer_fleet(&robot.capabilities.actions);
                            store.upsert(robot).await;
                        }
                    }
                    Err(e) => reject(store.clone(), &path, vec![e.to_string()]).await,
                },
                Err(rejected) => reject(store.clone(), &path, rejected.reasons).await,
            }
        }
        // Visualisation messages are high frequency and not needed for state tracking.
        Topic::Visualization => {
            store.count_message(true, Topic::Visualization).await;
        }
        _ => {}
    }
}

async fn reject(store: Arc<Store>, path: &TopicPath, reasons: Vec<String>) {
    warn!(topic = %path, ?reasons, "schema violation, message rejected");
    store
        .record_rejection(DlqEntry {
            topic: path.to_string(),
            topic_kind: path.topic,
            reasons: reasons.clone(),
        })
        .await;
    store
        .record(GatewayEvent::SchemaViolation {
            topic: path.to_string(),
            reasons,
        })
        .await;
}

fn placeholder(
    manufacturer: &str,
    serial_number: &str,
    version: &str,
) -> vda5050_types::CanonicalRobot {
    vda5050_types::CanonicalRobot {
        robot_id: format!("{manufacturer}:{serial_number}"),
        manufacturer: manufacturer.to_owned(),
        serial_number: serial_number.to_owned(),
        fleet: vda5050_types::canonical::FleetKind::default(),
        source: vda5050_types::canonical::Source::Robot,
        interface_name: "vda5050".to_owned(),
        major_version: 3,
        spec_version: version.to_owned(),
        map_id: None,
        position: None,
        power: None,
        operating_mode: "STARTUP".to_owned(),
        driving: false,
        paused: false,
        order_id: String::new(),
        action_states: Vec::new(),
        instant_action_states: Vec::new(),
        errors: Vec::new(),
        safety: None,
        connection: ConnectionState::Offline,
        last_seen: now_string(),
        capabilities: vda5050_types::canonical::Capabilities::default(),
    }
}

/// Current wall clock time as a VDA 5050 timestamp (millisecond precision).
#[must_use]
pub fn now_string() -> String {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default();
    let secs = now.as_secs();
    let millis = now.subsec_millis();
    // `cast_signed` needs Rust 1.87; keep the MSRV at 1.85 with an explicit conversion.
    #[allow(
        clippy::cast_possible_wrap,
        reason = "seconds since the epoch fit in i64 until the year 292 billion"
    )]
    let secs = i64::try_from(secs).unwrap_or(i64::MAX);
    let days = secs / 86_400;
    let rem = secs % 86_400;
    let (year, month, day) = civil_from_days(days);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{millis:03}Z",
        rem / 3_600,
        (rem % 3_600) / 60,
        rem % 60
    )
}

/// Inverse of `days_from_civil`.
fn civil_from_days(days: i64) -> (i64, i64, i64) {
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (if m <= 2 { y + 1 } else { y }, m, d)
}
