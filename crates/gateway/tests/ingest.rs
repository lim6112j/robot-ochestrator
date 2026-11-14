//! Unit tests for the MQTT ingest layer.
//!
//! These tests cover the parts of the gateway that do not need a live broker:
//! schema validation, v2 → v3 conversion and canonical normalisation.

use serde_json::json;

use gateway::store;
use vda5050_types::canonical::ConnectionState;
use vda5050_types::types::{EmergencyStop, OperatingMode};
use vda5050_types::{MessageValidator, Topic, TopicPath};

/// A minimal but schema-valid v3 `state` message.
fn valid_state() -> serde_json::Value {
    json!({
        "headerId": 1,
        "timestamp": "2026-01-01T00:00:00.000Z",
        "version": "3.0.0",
        "manufacturer": "acme",
        "serialNumber": "AMR-001",
        "orderId": "",
        "orderUpdateId": 0,
        "lastNodeId": "",
        "lastNodeSequenceId": 0,
        "nodeStates": [],
        "edgeStates": [],
        "driving": false,
        "mobileRobotPosition": {
            "x": 1.0, "y": 2.0, "theta": 0.5,
            "mapId": "plant-a/l1", "localized": true
        },
        "actionStates": [],
        "instantActionStates": [],
        "powerSupply": { "stateOfCharge": 88.5, "charging": false },
        "operatingMode": "AUTOMATIC",
        "errors": [],
        "safetyState": { "activeEmergencyStop": "NONE", "fieldViolation": false }
    })
}

#[test]
fn accepts_a_valid_state_message() {
    let validator = MessageValidator::new().expect("schemas compile");
    validator
        .validate(Topic::State, &valid_state())
        .expect("valid state");
}

#[test]
fn rejects_a_state_with_string_header_id() {
    let validator = MessageValidator::new().expect("schemas compile");
    let mut broken = valid_state();
    broken["headerId"] = json!("sim-AMR-001-000001");
    let reasons = validator
        .validate(Topic::State, &broken)
        .expect_err("headerId must be an integer");
    assert!(reasons.iter().any(|r| r.contains("integer")));
}

#[test]
fn rejects_a_state_with_null_instead_of_optional_object() {
    let validator = MessageValidator::new().expect("schemas compile");
    let mut broken = valid_state();
    broken["plannedPath"] = serde_json::Value::Null;
    validator
        .validate(Topic::State, &broken)
        .expect_err("null is not an object");
}

#[test]
fn rejects_a_state_missing_required_fields() {
    let validator = MessageValidator::new().expect("schemas compile");
    let broken = json!({ "headerId": 1 });
    validator
        .validate(Topic::State, &broken)
        .expect_err("missing required fields");
}

#[test]
fn v2_state_converts_to_canonical_v3_model() {
    // The gateway ingests v2 payloads without schema validation (v2 schemas are not vendored).
    let v2 = json!({
        "header": {
            "headerId": 7,
            "timestamp": "2026-01-01T00:00:01.500Z",
            "version": "2.1.0",
            "manufacturer": "otherco",
            "serialNumber": "AMR-009"
        },
        "orderId": "order-v2-1",
        "orderUpdateId": 0,
        "lastNodeId": "ST-07",
        "lastNodeSequenceId": 0,
        "nodeStates": [],
        "edgeStates": [],
        "driving": true,
        "agvPosition": {
            "x": 3.0, "y": 4.0, "theta": 1.0,
            "mapId": "plant-a/l1", "positionInitialized": true
        },
        "actionStates": [
            { "actionId": "a1", "actionType": "pick", "status": "RUNNING" }
        ],
        "batteryState": { "batteryCharge": 41.0, "charging": true },
        "operatingMode": "AUTOMATIC",
        "errors": [],
        "safetyState": { "eStop": "NONE", "fieldViolation": false }
    });

    let state: vda5050_types::v2::StateV2 = serde_json::from_value(v2).expect("v2 payload parses");
    let robot = state.to_canonical(ConnectionState::Online, "2026-01-01T00:00:02.000Z");

    assert_eq!(robot.robot_id, "otherco:AMR-009");
    assert_eq!(robot.major_version, 2);
    assert_eq!(robot.interface_name, "uomg");
    assert_eq!(robot.spec_version, "2.1.0");
    // v2 field names are mapped onto the v3 canonical model.
    let position = robot.position.as_ref().expect("position converted");
    assert_eq!(position.x, 3.0);
    assert!(position.localized, "positionInitialized maps to localized");
    let power = robot.power.as_ref().expect("power converted");
    assert_eq!(power.state_of_charge, 41.0);
    assert!(power.charging);
    assert_eq!(
        robot.operating_mode,
        format!("{:?}", OperatingMode::AUTOMATIC)
    );
    assert_eq!(robot.action_states.len(), 1);
    assert_eq!(
        robot.action_states[0].action_status,
        vda5050_types::types::ActionStatus::RUNNING
    );
    let safety = robot.safety.as_ref().expect("safety converted");
    assert_eq!(safety.active_emergency_stop, EmergencyStop::NONE);

    // The canonical model must be schema-valid v3 when re-serialised.
    let canonical = serde_json::to_value(&robot).expect("serialises");
    assert_eq!(canonical["majorVersion"], json!(2));
}

#[test]
fn v2_autostop_e_stop_maps_to_none() {
    let v2 = json!({
        "header": {
            "headerId": 8,
            "timestamp": "2026-01-01T00:00:00.000Z",
            "version": "2.0.0",
            "manufacturer": "m",
            "serialNumber": "s"
        },
        "driving": false,
        "actionStates": [],
        "safetyState": { "eStop": "AUTOACK", "fieldViolation": false }
    });
    let state: vda5050_types::v2::StateV2 = serde_json::from_value(v2).expect("parses");
    let robot = state.to_canonical(ConnectionState::Online, "2026-01-01T00:00:00.000Z");
    // v2's AUTOACK was removed in v3.
    assert_eq!(
        robot.safety.as_ref().expect("safety").active_emergency_stop,
        EmergencyStop::NONE
    );
}

#[test]
fn gateway_subscribes_to_the_v_prefixed_major_version() {
    assert_eq!(
        TopicPath::wildcard("vda5050", 3, Topic::State),
        "vda5050/v3/+/+/state"
    );
    assert_eq!(
        TopicPath::wildcard("uomg", 2, Topic::State),
        "uomg/v2/+/+/state"
    );
}

#[test]
fn gateway_subscribes_to_orders_so_the_counter_is_not_stuck_at_zero() {
    // `orders_seen` counts what the gateway *observes* on the wire, which is a
    // different number from `orders_published` (what the dashboard asked it to
    // send). The order branch in `count_message` was unreachable without this
    // subscription, so the counter read zero while missions were flowing.
    let subs = gateway::ingest::subscriptions("vda5050", 3, false);
    assert!(
        subs.iter().any(|s| s == "vda5050/v3/+/+/order"),
        "gateway must subscribe to orders, got {subs:?}"
    );
}

#[test]
fn the_v2_subscriptions_cover_the_same_topics_as_v3() {
    // A v2-only robot publishes orders on the `uomg` interface. Subscribing to
    // its state but not its order would leave half the picture missing.
    let subs = gateway::ingest::subscriptions("vda5050", 3, true);
    for expected in ["uomg/v2/+/+/order", "uomg/v2/+/+/state"] {
        assert!(
            subs.iter().any(|s| s == expected),
            "missing {expected}, got {subs:?}"
        );
    }
}

#[test]
fn a_valid_order_message_passes_validation() {
    // The ingest path validates before counting, so the order branch needs a
    // schema that actually accepts a well-formed order.
    let validator = MessageValidator::new().expect("schemas compile");
    validator
        .validate(Topic::Order, &valid_order())
        .expect("valid order");
}

/// A minimal but schema-valid v3 `order` message.
fn valid_order() -> serde_json::Value {
    json!({
        "headerId": 1,
        "timestamp": "2026-01-01T00:00:00.000Z",
        "version": "3.0.0",
        "manufacturer": "acme",
        "serialNumber": "AMR-001",
        "orderId": "order-1",
        "orderUpdateId": 0,
        "nodes": [
            { "nodeId": "ST-07", "sequenceId": 0, "released": true, "actions": [] }
        ],
        "edges": []
    })
}

#[tokio::test]
async fn one_order_message_counts_once() {
    // `handle_message` used to count every arriving message on entry *and*
    // again in the topic branch, so `messages_seen` and `orders_seen` came out
    // at twice the truth. The subscription fix alone would have turned
    // "always zero" into "always double", which is still wrong.
    let store = gateway::store::Store::new(30.0);
    let validator = MessageValidator::new().expect("schemas compile");
    let payload = serde_json::to_vec(&valid_order()).expect("serialises");

    gateway::ingest::handle_message(
        &validator,
        std::sync::Arc::clone(&store),
        "vda5050/v3/acme/AMR-001/order",
        &payload,
    )
    .await;

    let metrics = store.metrics().await;
    assert_eq!(metrics.orders_seen, 1, "one order, counted once");
    assert_eq!(metrics.messages_seen, 1, "one message, counted once");
    assert_eq!(metrics.messages_valid, 1);
}

#[tokio::test]
async fn a_rejected_order_is_still_counted_as_seen() {
    // A message that fails validation must not vanish from the counters: the
    // whole point of `messages_seen` is to show traffic the gateway could not
    // accept, and that is exactly what a malformed order is.
    let store = gateway::store::Store::new(30.0);
    let validator = MessageValidator::new().expect("schemas compile");
    let mut broken = valid_order();
    // `headerId` must be an integer; a string is a schema violation.
    broken["headerId"] = json!("not-a-number");
    let payload = serde_json::to_vec(&broken).expect("serialises");

    gateway::ingest::handle_message(
        &validator,
        std::sync::Arc::clone(&store),
        "vda5050/v3/acme/AMR-001/order",
        &payload,
    )
    .await;

    let metrics = store.metrics().await;
    assert_eq!(metrics.messages_seen, 1, "still seen");
    assert_eq!(metrics.messages_valid, 0, "but not valid");
    assert_eq!(metrics.messages_rejected, 1);
    // `orders_seen` counts messages on the order topic, valid or not — the
    // same relationship `orders_seen` has to `messages_seen`, where the
    // valid/invalid split is carried by `messages_valid` and
    // `messages_rejected` instead.
    assert_eq!(
        metrics.orders_seen, 1,
        "an order message is still an order message"
    );
}

#[tokio::test]
async fn a_message_on_an_unhandled_topic_is_counted_once() {
    // Topics with no branch fall through `_`. They must still register as
    // traffic, exactly once — the old entry-point count made them the only
    // messages that were counted correctly.
    let store = gateway::store::Store::new(30.0);
    let validator = MessageValidator::new().expect("schemas compile");

    gateway::ingest::handle_message(
        &validator,
        std::sync::Arc::clone(&store),
        "vda5050/v3/acme/AMR-001/zoneSet",
        br#"{"headerId": 1}"#,
    )
    .await;

    let metrics = store.metrics().await;
    assert_eq!(metrics.messages_seen, 1);
    assert_eq!(metrics.messages_valid, 0);
    assert_eq!(metrics.orders_seen, 0);
}

#[test]
fn fleet_kind_is_inferred_from_supported_actions() {
    use vda5050_types::canonical::FleetKind;
    assert_eq!(
        FleetKind::from_actions(&["pick".to_owned(), "drop".to_owned()]),
        FleetKind::Logistics
    );
    assert_eq!(
        FleetKind::from_actions(&["cleanZone".to_owned()]),
        FleetKind::Cleaning
    );
    assert_eq!(
        FleetKind::from_actions(&["patrol".to_owned()]),
        FleetKind::Surveillance
    );
    assert_eq!(
        FleetKind::from_actions(&["mysteryAction".to_owned()]),
        FleetKind::Other("unknown".to_owned())
    );
}

#[test]
fn connection_state_serialises_with_the_spec_spelling() {
    // The dashboard compares against "ONLINE".
    assert_eq!(
        serde_json::to_string(&ConnectionState::Online).expect("serialises"),
        "\"ONLINE\""
    );
    assert_eq!(
        serde_json::to_string(&ConnectionState::ConnectionBroken).expect("serialises"),
        "\"CONNECTION_BROKEN\""
    );
}

#[test]
fn heartbeat_timestamps_parse_and_compare() {
    // The store compares ISO-8601 timestamps to expire stale robots.
    let early = store::parse_ts("2026-01-01T00:00:00.000Z").expect("parses");
    let late = store::parse_ts("2026-01-01T00:00:10.000Z").expect("parses");
    assert!((late - early - 10.0).abs() < 1e-6);
    assert!(store::parse_ts("garbage").is_none());
}
