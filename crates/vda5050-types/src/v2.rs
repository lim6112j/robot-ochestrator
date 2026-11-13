//! Minimal VDA 5050 v2.x support (ingress only).
//!
//! Many fleets still speak v2.0/v2.1. The gateway converts those messages into
//! the v3 canonical model, so the rest of the system only ever sees v3 shapes.

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::canonical::{CanonicalRobot, ConnectionState};
use crate::types::{
    ActionState, ActionStatus, EmergencyStop, ErrorEntry, ErrorLevel, PowerSupply, SafetyState,
};

/// v2 `agvPosition` (renamed to `mobileRobotPosition` in v3).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AgvPosition {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub map_id: String,
    /// Renamed to `localized` in v3.
    pub position_initialized: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub localization_score: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deviation_range: Option<f64>,
}

/// v2 `batteryState` (renamed to `powerSupply` in v3).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BatteryState {
    pub battery_charge: f64,
    pub charging: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub battery_voltage: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reach: Option<f64>,
}

/// v2 `safetyState` (`eStop` renamed to `activeEmergencyStop` in v3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct V2SafetyState {
    pub e_stop: V2EmergencyStop,
    pub field_violation: bool,
}

/// v2 `safetyState.eStop`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub enum V2EmergencyStop {
    AUTOACK,
    MANUAL,
    REMOTE,
    NONE,
}

/// v2 `actionStates[]` entries (`status` renamed to `actionStatus` in v3).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct V2ActionState {
    pub action_id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub action_type: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub action_description: Option<String>,
    pub status: V2ActionStatus,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub result_description: Option<String>,
}

/// v2 `actionStates[].status`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub enum V2ActionStatus {
    WAITING,
    INITIALIZING,
    RUNNING,
    PAUSED,
    FINISHED,
    FAILED,
}

/// v2 `errors[]` entries.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct V2Error {
    pub error_type: String,
    #[serde(default)]
    pub error_references: Vec<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error_description: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error_level: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error_hint: Option<String>,
}

/// v2 `header` object.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct V2Header {
    pub header_id: u64,
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
}

/// The subset of the v2 `state` message the gateway consumes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct StateV2 {
    /// v2 field name; v3 renamed it to `headerId`.
    pub header: V2Header,
    #[serde(default)]
    pub order_id: String,
    #[serde(default)]
    pub order_update_id: u32,
    #[serde(default)]
    pub last_node_id: String,
    #[serde(default)]
    pub last_node_sequence_id: u32,
    #[serde(default)]
    pub node_states: Vec<Value>,
    #[serde(default)]
    pub edge_states: Vec<Value>,
    #[serde(default)]
    pub driving: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub agv_position: Option<AgvPosition>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub velocity: Option<Value>,
    #[serde(default)]
    pub loads: Vec<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub paused: Option<bool>,
    #[serde(default)]
    pub action_states: Vec<V2ActionState>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub battery_state: Option<BatteryState>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub operating_mode: Option<String>,
    #[serde(default)]
    pub errors: Vec<V2Error>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub safety_state: Option<V2SafetyState>,
}

fn map_error_level(raw: Option<&str>) -> ErrorLevel {
    match raw {
        Some("FATAL") => ErrorLevel::FATAL,
        Some("CRITICAL") => ErrorLevel::CRITICAL,
        Some("URGENT") => ErrorLevel::URGENT,
        _ => ErrorLevel::WARNING,
    }
}

fn map_emergency_stop(raw: V2EmergencyStop) -> EmergencyStop {
    match raw {
        V2EmergencyStop::AUTOACK | V2EmergencyStop::NONE => EmergencyStop::NONE,
        V2EmergencyStop::MANUAL => EmergencyStop::MANUAL,
        V2EmergencyStop::REMOTE => EmergencyStop::REMOTE,
    }
}

fn map_action_status(raw: V2ActionStatus) -> ActionStatus {
    match raw {
        V2ActionStatus::WAITING => ActionStatus::WAITING,
        V2ActionStatus::INITIALIZING => ActionStatus::INITIALIZING,
        V2ActionStatus::RUNNING => ActionStatus::RUNNING,
        V2ActionStatus::PAUSED => ActionStatus::PAUSED,
        // `RETRIABLE` was introduced in v3.
        V2ActionStatus::FINISHED => ActionStatus::FINISHED,
        V2ActionStatus::FAILED => ActionStatus::FAILED,
    }
}

impl StateV2 {
    /// Convert a v2 state message into the v3 canonical model.
    #[must_use]
    pub fn to_canonical(
        &self,
        connection: ConnectionState,
        now: impl Into<String>,
    ) -> CanonicalRobot {
        let position = self.agv_position.as_ref().map(|p| crate::types::Position {
            x: p.x,
            y: p.y,
            theta: p.theta,
            map_id: p.map_id.clone(),
            localized: p.position_initialized,
            localization_score: p.localization_score,
            deviation_range: p.deviation_range,
        });

        let power = self.battery_state.map(|b| PowerSupply {
            state_of_charge: b.battery_charge,
            charging: b.charging,
            battery_voltage: b.battery_voltage,
            battery_current: None,
            battery_health: None,
            range: b.reach,
        });

        let action_states = self
            .action_states
            .iter()
            .map(|a| ActionState {
                action_id: a.action_id.clone(),
                action_type: a.action_type.clone(),
                action_descriptor: a.action_description.clone(),
                action_status: map_action_status(a.status),
                action_result: a.result_description.clone(),
            })
            .collect();

        let errors = self
            .errors
            .iter()
            .map(|e| ErrorEntry {
                error_type: e.error_type.clone(),
                error_references: e.error_references.clone(),
                error_description: e.error_description.clone(),
                error_hint: e.error_hint.clone(),
                error_level: map_error_level(e.error_level.as_deref()),
            })
            .collect();

        let robot = CanonicalRobot {
            robot_id: format!("{}:{}", self.header.manufacturer, self.header.serial_number),
            manufacturer: self.header.manufacturer.clone(),
            serial_number: self.header.serial_number.clone(),
            fleet: crate::canonical::FleetKind::default(),
            source: crate::canonical::Source::Robot,
            interface_name: "uomg".to_owned(),
            major_version: 2,
            spec_version: self.header.version.clone(),
            map_id: position.as_ref().map(|p| p.map_id.clone()),
            position,
            power,
            operating_mode: self
                .operating_mode
                .clone()
                .unwrap_or_else(|| "MANUAL".to_owned()),
            driving: self.driving,
            paused: self.paused.unwrap_or(false),
            order_id: self.order_id.clone(),
            action_states,
            instant_action_states: Vec::new(),
            errors,
            safety: self.safety_state.map(|s| SafetyState {
                active_emergency_stop: map_emergency_stop(s.e_stop),
                field_violation: s.field_violation,
            }),
            connection,
            last_seen: now.into(),
            capabilities: crate::canonical::Capabilities::default(),
        };

        robot
    }
}
