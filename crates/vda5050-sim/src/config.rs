//! Simulator configuration (`--config sim.yaml`).

use std::collections::HashMap;
use std::path::Path;

use serde::Deserialize;

/// Supported behaviour profiles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Profile {
    /// Full v3.0.0 support.
    V3Full,
    /// v3.0.0 but with `zoneSet` unsupported and no zone requests.
    V3Partial,
    /// Speaks v2.x topics and field names (ingress conversion test target).
    V2Only,
}

/// Broker connection settings.
#[derive(Debug, Clone, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BrokerConfig {
    pub host: String,
    pub port: u16,
    pub tls: bool,
    pub username: Option<String>,
    pub password: Option<String>,
    pub client_prefix: String,
    pub interface: String,
    pub major_version: u8,
}

impl Default for BrokerConfig {
    fn default() -> Self {
        Self {
            host: "localhost".to_owned(),
            port: 1883,
            tls: false,
            username: None,
            password: None,
            client_prefix: "sim".to_owned(),
            interface: vda5050_types::DEFAULT_INTERFACE.to_owned(),
            major_version: vda5050_types::MAJOR_VERSION,
        }
    }
}

/// World (map) settings.
#[derive(Debug, Clone, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct WorldConfig {
    /// Path to the map graph JSON file.
    pub map: String,
    /// Simulation tick rate.
    pub tick_hz: f64,
    /// Explicit start node per robot (by fleet order); falls back to the first
    /// node when shorter than the fleet size.
    #[serde(default)]
    pub start_nodes: Vec<String>,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            map: "deploy/maps/plant-a.l1.json".to_owned(),
            tick_hz: 20.0,
            start_nodes: Vec::new(),
        }
    }
}

/// Battery model.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BatteryConfig {
    pub capacity_wh: f64,
    /// Average consumption while driving, in watts.
    pub drain_w: f64,
    /// Charging power, in watts.
    pub charge_w: f64,
    /// Start charge, in percent.
    pub start_percent: f64,
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
            capacity_wh: 500.0,
            drain_w: 80.0,
            charge_w: 300.0,
            start_percent: 90.0,
        }
    }
}

/// Fault injection for conformance testing.
#[derive(Debug, Clone, Copy, Default, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct FaultConfig {
    /// Probability of rejecting an order (0.0 - 1.0).
    pub reject_order_prob: f64,
    /// Probability of an action failing (0.0 - 1.0).
    pub action_fail_prob: f64,
    /// Terminate the robot (and publish `OFFLINE`) after N seconds.
    pub drop_connection_after_s: Option<f64>,
    /// Freeze state publishing after N seconds (simulates a silent robot).
    pub go_silent_after_s: Option<f64>,
}

/// One simulated fleet.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FleetConfig {
    pub manufacturer: String,
    pub serial_prefix: String,
    #[serde(default = "default_count")]
    pub count: usize,
    #[serde(default = "default_profile")]
    pub profile: Profile,
    #[serde(default = "default_publish_hz")]
    pub publish_hz: f64,
    #[serde(default)]
    pub actions: Vec<String>,
    #[serde(default)]
    pub battery: BatteryConfig,
    #[serde(default)]
    pub faults: FaultConfig,
}

fn default_count() -> usize {
    1
}

fn default_profile() -> Profile {
    Profile::V3Full
}

fn default_publish_hz() -> f64 {
    2.0
}

/// The whole simulator configuration.
#[derive(Debug, Clone, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct SimConfig {
    pub broker: BrokerConfig,
    pub world: WorldConfig,
    pub fleet: HashMap<String, FleetConfig>,
    /// Fixed seed for deterministic runs.
    pub seed: u64,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            broker: BrokerConfig::default(),
            world: WorldConfig::default(),
            fleet: HashMap::new(),
            seed: 42,
        }
    }
}

impl SimConfig {
    /// Load configuration from a YAML file.
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let raw = std::fs::read_to_string(path).map_err(|source| ConfigError::Io {
            path: path.display().to_string(),
            source,
        })?;
        Self::from_yaml(&raw)
    }

    /// Parse configuration from a YAML string.
    pub fn from_yaml(raw: &str) -> Result<Self, ConfigError> {
        serde_yaml::from_str(raw).map_err(ConfigError::Yaml)
    }

    /// Flatten the fleet map into a list of robot definitions.
    #[must_use]
    pub fn robots(&self) -> Vec<RobotSpec> {
        let mut out = Vec::new();
        for (fleet_name, fleet) in &self.fleet {
            for i in 1..=fleet.count {
                out.push(RobotSpec {
                    fleet: fleet_name.clone(),
                    manufacturer: fleet.manufacturer.clone(),
                    serial_number: format!("{}-{:03}", fleet.serial_prefix, i),
                    profile: fleet.profile,
                    publish_hz: fleet.publish_hz,
                    actions: fleet.actions.clone(),
                    battery: fleet.battery,
                    faults: fleet.faults,
                });
            }
        }
        out.sort_by(|a, b| a.serial_number.cmp(&b.serial_number));
        out
    }
}

/// One virtual robot.
#[derive(Debug, Clone)]
pub struct RobotSpec {
    pub fleet: String,
    pub manufacturer: String,
    pub serial_number: String,
    pub profile: Profile,
    pub publish_hz: f64,
    pub actions: Vec<String>,
    pub battery: BatteryConfig,
    pub faults: FaultConfig,
}

/// Configuration errors.
#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    #[error("cannot read config {path}: {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("invalid config: {0}")]
    Yaml(#[from] serde_yaml::Error),
}

#[cfg(test)]
mod tests {
    use super::*;

    const YAML: &str = r"
broker:
  host: broker.local
  port: 1884
world:
  map: deploy/maps/plant-a.l1.json
  tick_hz: 20
fleet:
  logistics:
    manufacturer: acme
    serialPrefix: AMR
    count: 2
    profile: v3-full
    publish_hz: 2.0
    actions: [pick, drop]
  surveillance:
    manufacturer: watchco
    serialPrefix: PTZ
    count: 1
    profile: v2-only
    actions: [patrol]
seed: 7
";

    #[test]
    fn parses_yaml_and_flattens_robots() {
        let cfg = SimConfig::from_yaml(YAML).expect("valid yaml");
        assert_eq!(cfg.broker.host, "broker.local");
        assert_eq!(cfg.broker.port, 1884);
        assert_eq!(cfg.seed, 7);
        let robots = cfg.robots();
        assert_eq!(robots.len(), 3);
        assert_eq!(robots[0].serial_number, "AMR-001");
        assert_eq!(robots[1].serial_number, "AMR-002");
        assert_eq!(robots[2].serial_number, "PTZ-001");
        assert_eq!(robots[2].profile, Profile::V2Only);
        assert_eq!(robots[0].fleet, "logistics");
    }

    #[test]
    fn defaults_are_sane() {
        let cfg = SimConfig::from_yaml("fleet: {}\n").expect("valid yaml");
        assert_eq!(cfg.broker.port, 1883);
        assert_eq!(cfg.world.tick_hz, 20.0);
        assert!(cfg.robots().is_empty());
    }
}
