//! VDA 5050 v3.0.0 message types, topic handling and JSON Schema validation.
//!
//! The schemas are vendored from the official `vda5050/vda5050` repository
//! (tag `3.0.0`) and embedded into the binary, so validation works offline.
//! See `schemas/vda5050/3.0.0/PATCHES.md` for the local syntax fixes.

use std::collections::HashMap;
use std::fmt;
use std::str::FromStr;

use jsonschema::{ValidationError, Validator};
use serde_json::Value;
use thiserror::Error;

pub mod canonical;
pub mod types;
pub mod v2;

pub use canonical::{CanonicalRobot, ConnectionState};
pub use types::*;

/// Default interface name used by VDA 5050 v3 examples and this implementation.
pub const DEFAULT_INTERFACE: &str = "vda5050";
/// Major version implemented by this crate.
pub const MAJOR_VERSION: u8 = 3;
/// Full spec version of the vendored schemas.
pub const SPEC_VERSION: &str = "3.0.0";

const SCHEMA_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../schemas/vda5050/3.0.0");

/// Communication topics defined by VDA 5050.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Topic {
    Order,
    InstantActions,
    State,
    Visualization,
    Connection,
    Factsheet,
    ZoneSet,
    Responses,
}

impl Topic {
    /// All topics, in the order used by the specification table.
    pub const ALL: [Topic; 8] = [
        Topic::Order,
        Topic::InstantActions,
        Topic::State,
        Topic::Visualization,
        Topic::Connection,
        Topic::Factsheet,
        Topic::ZoneSet,
        Topic::Responses,
    ];

    /// Topic name as it appears as the last MQTT topic level.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Topic::Order => "order",
            Topic::InstantActions => "instantActions",
            Topic::State => "state",
            Topic::Visualization => "visualization",
            Topic::Connection => "connection",
            Topic::Factsheet => "factsheet",
            Topic::ZoneSet => "zoneSet",
            Topic::Responses => "responses",
        }
    }

    /// Who publishes this topic.
    #[must_use]
    pub const fn publisher(self) -> Publisher {
        match self {
            Topic::Order | Topic::InstantActions | Topic::ZoneSet | Topic::Responses => {
                Publisher::FleetControl
            }
            // The broker publishes `connection` via MQTT last will.
            Topic::Connection => Publisher::BrokerOrRobot,
            _ => Publisher::Robot,
        }
    }

    /// `QoS` level required by the specification.
    #[must_use]
    pub const fn qos(self) -> QoS {
        match self {
            Topic::Connection => QoS::AtLeastOnce,
            _ => QoS::AtMostOnce,
        }
    }
}

impl fmt::Display for Topic {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl FromStr for Topic {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Topic::ALL
            .into_iter()
            .find(|t| t.as_str() == s)
            .ok_or_else(|| Error::UnknownTopic(s.to_owned()))
    }
}

/// Direction of a topic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Publisher {
    FleetControl,
    Robot,
    BrokerOrRobot,
}

/// MQTT quality of service.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QoS {
    AtMostOnce,
    AtLeastOnce,
}

/// A parsed VDA 5050 MQTT topic.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicPath {
    pub interface: String,
    pub major_version: u8,
    pub manufacturer: String,
    pub serial_number: String,
    pub topic: Topic,
}

impl TopicPath {
    /// Build a topic path from its parts.
    #[must_use]
    pub fn new(
        interface: impl Into<String>,
        major_version: u8,
        manufacturer: impl Into<String>,
        serial_number: impl Into<String>,
        topic: Topic,
    ) -> Self {
        Self {
            interface: interface.into(),
            major_version,
            manufacturer: manufacturer.into(),
            serial_number: serial_number.into(),
            topic,
        }
    }

    /// Parse `interfaceName/majorVersion/manufacturer/serialNumber/topic`.
    pub fn parse(raw: &str) -> Result<Self, Error> {
        let parts: Vec<&str> = raw.split('/').collect();
        if parts.len() != 5 {
            return Err(Error::MalformedTopic(raw.to_owned()));
        }
        let major_version = parts[1]
            .strip_prefix('v')
            .and_then(|v| v.parse::<u8>().ok())
            .ok_or_else(|| Error::MalformedTopic(raw.to_owned()))?;
        Ok(Self {
            interface: parts[0].to_owned(),
            major_version,
            manufacturer: parts[2].to_owned(),
            serial_number: parts[3].to_owned(),
            topic: parts[4].parse()?,
        })
    }

    /// Subscribe pattern that matches every robot of one interface and major version.
    ///
    /// The major version level is the version number preceded by `v`, e.g. `v3`.
    #[must_use]
    pub fn wildcard(interface: &str, major_version: u8, topic: Topic) -> String {
        format!("{interface}/v{major_version}/+/+/{topic}")
    }

    /// Robot identity as `manufacturer:serialNumber`.
    #[must_use]
    pub fn robot_id(&self) -> String {
        format!("{}:{}", self.manufacturer, self.serial_number)
    }
}

impl fmt::Display for TopicPath {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}/v{}/{}/{}/{}",
            self.interface, self.major_version, self.manufacturer, self.serial_number, self.topic
        )
    }
}

/// Errors produced by this crate.
#[derive(Debug, Error)]
pub enum Error {
    #[error("unknown VDA 5050 topic: {0}")]
    UnknownTopic(String),
    #[error("malformed VDA 5050 topic: {0}")]
    MalformedTopic(String),
    #[error("failed to load embedded schema for {topic}: {source}")]
    SchemaLoad {
        topic: Topic,
        #[source]
        source: serde_json::Error,
    },
    #[error("failed to compile schema for {topic}: {source}")]
    SchemaCompile {
        topic: Topic,
        #[source]
        source: ValidationError<'static>,
    },
    #[error("message is not valid JSON: {0}")]
    Json(#[from] serde_json::Error),
    #[error("{topic} message violates schema: {reasons:?}")]
    SchemaViolation { topic: Topic, reasons: Vec<String> },
}

/// A message that failed schema validation, kept for the DLQ.
#[derive(Debug, Clone)]
pub struct Rejected {
    pub topic: Topic,
    pub topic_path: String,
    pub reasons: Vec<String>,
    pub payload: Value,
}

/// Validates VDA 5050 messages against the official JSON schemas.
#[derive(Debug)]
pub struct MessageValidator {
    validators: HashMap<Topic, Validator>,
}

impl MessageValidator {
    /// Compile every embedded schema.
    pub fn new() -> Result<Self, Error> {
        let mut validators = HashMap::new();
        for topic in Topic::ALL {
            let file = format!("{SCHEMA_DIR}/{}.schema", topic.as_str());
            let raw = std::fs::read_to_string(&file).map_err(|e| Error::SchemaLoad {
                topic,
                source: serde_json::Error::io(e),
            })?;
            let schema: Value =
                serde_json::from_str(&raw).map_err(|source| Error::SchemaLoad { topic, source })?;
            let validator =
                Validator::new(&schema).map_err(|source| Error::SchemaCompile { topic, source })?;
            validators.insert(topic, validator);
        }
        Ok(Self { validators })
    }

    /// Validate an already parsed message.
    pub fn validate(&self, topic: Topic, value: &Value) -> Result<(), Vec<String>> {
        let validator = self
            .validators
            .get(&topic)
            .expect("validator map is complete");
        if validator.is_valid(value) {
            return Ok(());
        }
        let reasons: Vec<String> = validator
            .iter_errors(value)
            .map(|e| e.to_string())
            .collect();
        Err(reasons)
    }

    /// Parse and validate raw MQTT payload bytes.
    pub fn validate_bytes(&self, topic: Topic, bytes: &[u8]) -> Result<Value, Rejected> {
        let value: Value = match serde_json::from_slice(bytes) {
            Ok(v) => v,
            Err(e) => {
                return Err(Rejected {
                    topic,
                    topic_path: String::new(),
                    reasons: vec![format!("invalid JSON: {e}")],
                    payload: Value::Null,
                })
            }
        };
        self.validate_value(topic, value)
    }

    /// Validate an already parsed message, wrapping failures in [`Rejected`].
    pub fn validate_value(&self, topic: Topic, value: Value) -> Result<Value, Rejected> {
        match self.validate(topic, &value) {
            Ok(()) => Ok(value),
            Err(reasons) => Err(Rejected {
                topic,
                topic_path: String::new(),
                reasons,
                payload: value,
            }),
        }
    }
}

impl Default for MessageValidator {
    fn default() -> Self {
        Self::new().expect("embedded schemas must compile")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn topic_round_trip() {
        for topic in Topic::ALL {
            let parsed = Topic::from_str(topic.as_str()).expect("topic parses");
            assert_eq!(parsed, topic);
        }
    }

    #[test]
    fn topic_path_parse_and_display() {
        let raw = "vda5050/v3/acme/AMR-001/state";
        let path = TopicPath::parse(raw).expect("valid topic");
        assert_eq!(path.manufacturer, "acme");
        assert_eq!(path.serial_number, "AMR-001");
        assert_eq!(path.topic, Topic::State);
        assert_eq!(path.robot_id(), "acme:AMR-001");
        assert_eq!(path.to_string(), raw);
    }

    #[test]
    fn topic_path_rejects_wrong_level_count() {
        assert!(TopicPath::parse("vda5050/v3/acme/state").is_err());
        // major version must be prefixed with "v"
        assert!(TopicPath::parse("vda5050/3/acme/AMR-1/state").is_err());
        assert!(TopicPath::parse("vda5050/v3/acme/AMR-1/bogus").is_err());
    }

    #[test]
    fn wildcard_pattern_matches_all_robots() {
        let pattern = TopicPath::wildcard(DEFAULT_INTERFACE, MAJOR_VERSION, Topic::State);
        assert_eq!(pattern, "vda5050/v3/+/+/state");
        assert!(TopicPath::parse("vda5050/v3/acme/AMR-001/state").is_ok());
        assert!(TopicPath::parse("vda5050/v3/a/b/state").is_ok());
    }

    #[test]
    fn connection_uses_qos1() {
        assert_eq!(Topic::Connection.qos(), QoS::AtLeastOnce);
        assert_eq!(Topic::State.qos(), QoS::AtMostOnce);
    }
}
