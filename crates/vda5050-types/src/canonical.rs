//! Internal canonical model.
//!
//! Every robot — regardless of manufacturer or spec version — is normalised
//! into [`CanonicalRobot`] before it reaches the BFF and the UI.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use crate::types::{Factsheet, PowerSupply, SafetyState, State};

/// Connection state as tracked by the gateway.
///
/// Serialised with the spec spelling (`ONLINE`, `OFFLINE`, `CONNECTION_BROKEN`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConnectionState {
    #[serde(rename = "ONLINE")]
    Online,
    #[serde(rename = "OFFLINE")]
    Offline,
    #[serde(rename = "CONNECTION_BROKEN")]
    ConnectionBroken,
}

impl ConnectionState {
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            ConnectionState::Online => "ONLINE",
            ConnectionState::Offline => "OFFLINE",
            ConnectionState::ConnectionBroken => "CONNECTION_BROKEN",
        }
    }
}

impl From<crate::types::ConnectionState> for ConnectionState {
    fn from(value: crate::types::ConnectionState) -> Self {
        match value {
            crate::types::ConnectionState::ONLINE => ConnectionState::Online,
            crate::types::ConnectionState::OFFLINE => ConnectionState::Offline,
            crate::types::ConnectionState::CONNECTION_BROKEN => ConnectionState::ConnectionBroken,
        }
    }
}

/// Where the data came from — real robot or simulator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Source {
    Robot,
    Simulator,
}

/// Fleet category used for grouping in the UI.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum FleetKind {
    Logistics,
    Cleaning,
    Surveillance,
    Other(String),
}

impl FleetKind {
    /// Classify a fleet from the actions its robots support.
    ///
    /// VDA 5050 has no fleet-category field, so the gateway infers it from the
    /// `factsheet.protocolFeatures.mobileRobotActions` list.
    #[must_use]
    pub fn from_actions(actions: &[String]) -> Self {
        let has = |needles: &[&str]| {
            actions
                .iter()
                .any(|a| needles.iter().any(|n| a.eq_ignore_ascii_case(n)))
        };
        if has(&["cleanZone", "clean", "vacuum", "scrub"]) {
            FleetKind::Cleaning
        } else if has(&["patrol", "surveillance", "inspect", "monitor"]) {
            FleetKind::Surveillance
        } else if has(&["pick", "drop", "initPosition", "charge", "load", "unload"]) {
            FleetKind::Logistics
        } else {
            FleetKind::Other("unknown".to_owned())
        }
    }
}

impl Default for FleetKind {
    fn default() -> Self {
        FleetKind::Other("unknown".to_owned())
    }
}

/// Capabilities discovered from the `factsheet`.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Capabilities {
    #[serde(default)]
    pub actions: BTreeSet<String>,
    #[serde(default)]
    pub series_name: Option<String>,
    #[serde(default)]
    pub kinematics: Option<String>,
    #[serde(default)]
    pub max_load_mass_kg: Option<i64>,
    /// Whether the robot supports `zoneSet` (optional topic).
    #[serde(default)]
    pub zones: bool,
}

/// The normalised view of one mobile robot.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CanonicalRobot {
    pub robot_id: String,
    pub manufacturer: String,
    pub serial_number: String,
    pub fleet: FleetKind,
    pub source: Source,
    pub interface_name: String,
    pub major_version: u8,
    pub spec_version: String,
    pub map_id: Option<String>,
    pub position: Option<crate::types::Position>,
    pub power: Option<PowerSupply>,
    pub operating_mode: String,
    pub driving: bool,
    pub paused: bool,
    pub order_id: String,
    pub action_states: Vec<crate::types::ActionState>,
    pub instant_action_states: Vec<crate::types::ActionState>,
    pub errors: Vec<crate::types::ErrorEntry>,
    pub safety: Option<SafetyState>,
    pub connection: ConnectionState,
    pub last_seen: String,
    pub capabilities: Capabilities,
}

impl CanonicalRobot {
    /// Build a canonical entry from a `state` message.
    #[must_use]
    pub fn from_state(state: &State, connection: ConnectionState, now: impl Into<String>) -> Self {
        Self {
            robot_id: format!("{}:{}", state.manufacturer, state.serial_number),
            manufacturer: state.manufacturer.clone(),
            serial_number: state.serial_number.clone(),
            fleet: FleetKind::default(),
            source: Source::Robot,
            interface_name: crate::DEFAULT_INTERFACE.to_owned(),
            major_version: crate::MAJOR_VERSION,
            spec_version: state.version.clone(),
            map_id: state
                .mobile_robot_position
                .as_ref()
                .map(|p| p.map_id.clone()),
            position: state.mobile_robot_position.clone(),
            power: Some(state.power_supply),
            operating_mode: format!("{:?}", state.operating_mode),
            driving: state.driving,
            paused: state.paused,
            order_id: state.order_id.clone(),
            action_states: state.action_states.clone(),
            instant_action_states: state.instant_action_states.clone(),
            errors: state.errors.clone(),
            safety: Some(state.safety_state),
            connection,
            last_seen: now.into(),
            capabilities: Capabilities::default(),
        }
    }

    /// Merge capabilities from a `factsheet`.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_precision_loss,
        reason = "load mass in kg is rounded to the nearest integer for display"
    )]
    pub fn apply_factsheet(&mut self, factsheet: &Factsheet) {
        let mut caps = Capabilities {
            actions: factsheet.supported_actions().into_iter().collect(),
            ..Capabilities::default()
        };
        if let Some(ts) = &factsheet.type_specification {
            caps.series_name.clone_from(&ts.series_name);
            caps.kinematics.clone_from(&ts.mobile_robot_kinematics);
            caps.max_load_mass_kg = ts
                .maximum_load_mass
                .map(|m| m.clamp(0.0, i64::MAX as f64) as i64);
        }
        // Optional parameters tell us whether the robot implements zone sets.
        if let Some(features) = &factsheet.protocol_features {
            caps.zones = features
                .optional_parameters
                .iter()
                .any(|p| p.get("parameter").and_then(|v| v.as_str()) == Some("zoneSet"));
        }
        self.capabilities = caps;
    }

    /// True when the robot currently reports any error.
    #[must_use]
    pub fn has_errors(&self) -> bool {
        !self.errors.is_empty()
    }

    /// Highest error level, as a comparable rank (higher is worse).
    #[must_use]
    pub fn worst_error_level(&self) -> Option<&'static str> {
        self.errors
            .iter()
            .max_by_key(|e| e.error_level.rank())
            .map(|e| e.error_level.as_str())
    }
}
