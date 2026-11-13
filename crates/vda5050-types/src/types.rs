//! VDA 5050 v3.0.0 wire types.
//!
//! Field names follow the vendored schemas exactly (`mobileRobotPosition`,
//! `powerSupply`, `headerId`, ...). Only the fields this project consumes are
//! modelled; unknown fields are preserved by re-serialising through
//! `serde_json::Value` where necessary.

use serde::{Deserialize, Serialize};
use serde_json::Value;

fn default_false() -> bool {
    false
}

fn default_u32() -> u32 {
    0
}

/// Common header of every VDA 5050 message (v3 field name: `headerId`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Header {
    pub header_id: u64,
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
}

/// Position of a mobile robot (`mobileRobotPosition`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub map_id: String,
    pub localized: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub localization_score: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deviation_range: Option<f64>,
}

/// Battery information (`powerSupply`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PowerSupply {
    pub state_of_charge: f64,
    pub charging: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub battery_voltage: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub battery_current: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub battery_health: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub range: Option<f64>,
}

/// Operating modes of a mobile robot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub enum OperatingMode {
    STARTUP,
    AUTOMATIC,
    SEMIAUTOMATIC,
    INTERVENED,
    MANUAL,
    SERVICE,
    TEACH_IN,
}

/// Safety state (`safetyState`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SafetyState {
    pub active_emergency_stop: EmergencyStop,
    pub field_violation: bool,
}

/// `safetyState.activeEmergencyStop`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub enum EmergencyStop {
    MANUAL,
    REMOTE,
    NONE,
}

/// Error severity (`errorLevel`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ErrorLevel {
    WARNING,
    URGENT,
    CRITICAL,
    FATAL,
}

impl ErrorLevel {
    /// Severity rank, higher is worse.
    #[must_use]
    pub const fn rank(self) -> u8 {
        match self {
            ErrorLevel::WARNING => 0,
            ErrorLevel::URGENT => 1,
            ErrorLevel::CRITICAL => 2,
            ErrorLevel::FATAL => 3,
        }
    }

    /// Spec spelling.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            ErrorLevel::WARNING => "WARNING",
            ErrorLevel::URGENT => "URGENT",
            ErrorLevel::CRITICAL => "CRITICAL",
            ErrorLevel::FATAL => "FATAL",
        }
    }
}

/// An entry of `state.errors`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ErrorEntry {
    pub error_type: String,
    #[serde(default)]
    pub error_references: Vec<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error_description: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error_hint: Option<String>,
    pub error_level: ErrorLevel,
}

/// `actionStates[]` / `instantActionStates[]` entries.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ActionState {
    pub action_id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub action_type: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub action_descriptor: Option<String>,
    pub action_status: ActionStatus,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub action_result: Option<String>,
}

/// `actionStates[].actionStatus`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub enum ActionStatus {
    WAITING,
    INITIALIZING,
    RUNNING,
    PAUSED,
    RETRIABLE,
    FINISHED,
    FAILED,
}

/// `nodeStates[]` entries.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NodeState {
    pub node_id: String,
    pub sequence_id: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub node_descriptor: Option<String>,
    pub released: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub node_position: Option<Position>,
}

/// `edgeStates[]` entries.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EdgeState {
    pub edge_id: String,
    pub sequence_id: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub edge_descriptor: Option<String>,
    pub released: bool,
}

/// The `state` message.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct State {
    pub header_id: u64,
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
    #[serde(default)]
    pub maps: Vec<Value>,
    #[serde(default)]
    pub zone_sets: Vec<Value>,
    pub order_id: String,
    #[serde(default = "default_u32")]
    pub order_update_id: u32,
    #[serde(default)]
    pub last_node_id: String,
    #[serde(default = "default_u32")]
    pub last_node_sequence_id: u32,
    pub node_states: Vec<NodeState>,
    pub edge_states: Vec<EdgeState>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub planned_path: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub intermediate_path: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mobile_robot_position: Option<Position>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub velocity: Option<Value>,
    #[serde(default)]
    pub loads: Vec<Value>,
    pub driving: bool,
    #[serde(default = "default_false")]
    pub paused: bool,
    #[serde(default)]
    pub new_base_request: bool,
    #[serde(default)]
    pub zone_requests: Vec<Value>,
    #[serde(default)]
    pub edge_requests: Vec<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance_since_last_node: Option<f64>,
    pub action_states: Vec<ActionState>,
    pub instant_action_states: Vec<ActionState>,
    #[serde(default)]
    pub zone_action_states: Vec<ActionState>,
    pub power_supply: PowerSupply,
    pub operating_mode: OperatingMode,
    pub errors: Vec<ErrorEntry>,
    #[serde(default)]
    pub information: Vec<Value>,
    pub safety_state: SafetyState,
}

impl State {
    /// Highest error level currently reported, if any.
    #[must_use]
    pub fn worst_error_level(&self) -> Option<ErrorLevel> {
        self.errors.iter().map(|e| e.error_level).max()
    }

    /// True when the robot reported an active order.
    #[must_use]
    pub fn has_active_order(&self) -> bool {
        !self.order_id.is_empty()
    }
}

/// Blocking type of an action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BlockingType {
    NONE,
    SOFT,
    SINGLE,
    HARD,
}

/// An action inside `order.nodes[].actions` / `order.edges[].actions`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Action {
    pub action_type: String,
    pub action_id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub action_descriptor: Option<String>,
    pub blocking_type: BlockingType,
    #[serde(default)]
    pub action_parameters: Vec<ActionParameter>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retriable: Option<bool>,
}

/// A key/value action parameter.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ActionParameter {
    pub key: String,
    pub value: Value,
}

/// A node of an order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Node {
    pub node_id: String,
    pub sequence_id: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub node_descriptor: Option<String>,
    pub released: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub node_position: Option<Position>,
    pub actions: Vec<Action>,
}

/// An edge of an order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Edge {
    pub edge_id: String,
    pub sequence_id: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub edge_descriptor: Option<String>,
    pub released: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub maximum_speed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub orientation: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub direction: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reach_orientation_before_entering: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub length: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub start_node_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end_node_id: Option<String>,
    #[serde(default)]
    pub actions: Vec<Action>,
}

/// The `order` message.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Order {
    pub header_id: u64,
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
    pub order_id: String,
    pub order_update_id: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub order_description: Option<String>,
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
}

/// An entry of `instantActions.actions`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct InstantAction {
    pub action_type: String,
    pub action_id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub action_descriptor: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub blocking_type: Option<BlockingType>,
    #[serde(default)]
    pub action_parameters: Vec<ActionParameter>,
}

/// The `instantActions` message.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct InstantActions {
    pub header_id: u64,
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
    pub actions: Vec<InstantAction>,
}

/// `connection.connectionState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub enum ConnectionState {
    ONLINE,
    OFFLINE,
    CONNECTION_BROKEN,
}

/// The `connection` message (published by the robot, or by the broker as last will).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Connection {
    pub header_id: u64,
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
    pub connection_state: ConnectionState,
}

/// The `visualization` message.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Visualization {
    pub header_id: u64,
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
    pub reference_state_header_id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub planned_path: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub intermediate_path: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mobile_robot_position: Option<Position>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub velocity: Option<Value>,
}

/// A lease answer in `responses.responses`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct LeaseResponse {
    pub request_id: String,
    pub grant_type: GrantType,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lease_expiry: Option<String>,
}

/// `responses.responses[].grantType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GrantType {
    GRANTED,
    QUEUED,
    REVOKED,
    REJECTED,
}

/// The `responses` message (zone/edge lease answers only in v3.0.0).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Responses {
    pub header_id: u64,
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
    pub responses: Vec<LeaseResponse>,
}

/// Selected fields of the `factsheet` message.
///
/// The full factsheet is large and mostly static; only the parts the control
/// system needs (identity, supported actions, load handling) are modelled.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Factsheet {
    pub header_id: u64,
    pub timestamp: String,
    pub version: String,
    pub manufacturer: String,
    pub serial_number: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub type_specification: Option<TypeSpecification>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub protocol_features: Option<ProtocolFeatures>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub load_specification: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub physical_parameters: Option<Value>,
}

/// `factsheet.typeSpecification` (selected fields).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TypeSpecification {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub series_name: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub series_description: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mobile_robot_kinematics: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mobile_robot_class: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub maximum_load_mass: Option<f64>,
}

/// `factsheet.protocolFeatures` (selected fields).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProtocolFeatures {
    #[serde(default)]
    pub optional_parameters: Vec<Value>,
    #[serde(default)]
    pub mobile_robot_actions: Vec<MobileRobotAction>,
}

/// One entry of `protocolFeatures.mobileRobotActions`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MobileRobotAction {
    pub action_type: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub action_description: Option<String>,
    #[serde(default)]
    pub blocking_types: Vec<BlockingType>,
    #[serde(default)]
    pub action_parameters: Vec<ActionParameter>,
}

impl Factsheet {
    /// Action types this robot claims to support.
    #[must_use]
    pub fn supported_actions(&self) -> Vec<String> {
        self.protocol_features
            .as_ref()
            .map(|f| {
                f.mobile_robot_actions
                    .iter()
                    .map(|a| a.action_type.clone())
                    .collect()
            })
            .unwrap_or_default()
    }
}
