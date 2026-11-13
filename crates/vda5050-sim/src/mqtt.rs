//! MQTT 5 connection helpers for the simulator.

use rumqttc::{AsyncClient, Event, EventLoop, LastWill, MqttOptions, QoS};
use serde_json::Value;
use vda5050_types::Topic;

use crate::config::BrokerConfig;

/// Errors from the MQTT layer.
#[derive(Debug, thiserror::Error)]
pub enum MqttError {
    #[error("mqtt client error: {0}")]
    Client(#[from] rumqttc::ClientError),
    #[error("mqtt connection error: {0}")]
    Connection(#[from] Box<rumqttc::ConnectionError>),
    #[error("cannot serialise message: {0}")]
    Serialization(String),
}

/// A connected robot client together with its event loop.
pub struct RobotLink {
    pub client: AsyncClient,
    pub eventloop: EventLoop,
}

impl RobotLink {
    /// Connect using the given options, registering `will` as the MQTT last will.
    #[must_use]
    pub fn new(options: MqttOptions, will: LastWill, capacity: usize) -> Self {
        let mut options = options;
        options.set_last_will(will);
        let (client, eventloop) = AsyncClient::new(options, capacity);
        Self { client, eventloop }
    }
}

/// Build MQTT 5 options for one robot.
#[must_use]
pub fn build_options(client_id: &str, broker: &BrokerConfig) -> MqttOptions {
    let mut options = MqttOptions::new(client_id, broker.host.clone(), broker.port);
    options.set_keep_alive(std::time::Duration::from_secs(5));
    if let (Some(user), Some(pass)) = (&broker.username, &broker.password) {
        options.set_credentials(user.clone(), pass.clone());
    }
    options
}

/// Last will payload: the broker publishes this when the robot disappears.
#[must_use]
pub fn connection_will(topic_path: &str, payload: &Value) -> LastWill {
    LastWill::new(
        topic_path,
        serde_json::to_vec(payload).unwrap_or_default(),
        QoS::AtLeastOnce,
        false,
    )
}

/// Publish a JSON payload.
pub async fn publish_json(
    client: &AsyncClient,
    topic: &str,
    topic_kind: Topic,
    retain: bool,
    payload: &Value,
) -> Result<(), MqttError> {
    let bytes = serde_json::to_vec(payload).map_err(|e| MqttError::Serialization(e.to_string()))?;
    client
        .publish(topic, qos(topic_kind), retain, bytes)
        .await?;
    Ok(())
}

/// `QoS` level required by the specification for a topic.
#[must_use]
pub const fn qos(topic: Topic) -> QoS {
    match topic {
        Topic::Connection => QoS::AtLeastOnce,
        _ => QoS::AtMostOnce,
    }
}

/// Extract the topic and payload of an incoming publish.
#[must_use]
pub fn incoming_publish(event: &Event) -> Option<(String, Vec<u8>)> {
    if let Event::Incoming(rumqttc::Incoming::Publish(publish)) = event {
        Some((publish.topic.clone(), publish.payload.to_vec()))
    } else {
        None
    }
}
