//! Integration tests for [`MqttRobotAdapter`] through the real trait.
//!
//! These exercise the adapter the way the gateway will: observe state, then
//! answer questions about it. No broker is involved — the point is the
//! translation from what a robot reports to what the trait promises.

use robot_adapter::mqtt::MqttRobotAdapter;
use robot_adapter::{AdapterError, ResourceKind, RobotAdapter};

fn state(
    manufacturer: &str,
    serial: &str,
    node: &str,
    driving: bool,
) -> vda5050_types::types::State {
    serde_json::from_value(serde_json::json!({
        "headerId": 1,
        "timestamp": "2026-01-01T00:00:00.000Z",
        "version": "3.0.0",
        "manufacturer": manufacturer,
        "serialNumber": serial,
        "orderId": "o1",
        "orderUpdateId": 0,
        "lastNodeId": node,
        "lastNodeSequenceId": 0,
        "nodeStates": [{
            "nodeId": node, "sequenceId": 0, "released": true, "actions": []
        }],
        "edgeStates": [],
        "driving": driving,
        "actionStates": [],
        "instantActionStates": [],
        "powerSupply": { "stateOfCharge": 55.0, "charging": false },
        "operatingMode": "AUTOMATIC",
        "errors": [],
        "safetyState": { "activeEmergencyStop": "NONE", "fieldViolation": false }
    }))
    .expect("valid state")
}

#[tokio::test]
async fn an_adapter_answers_for_only_its_own_manufacturer() {
    let adapter = MqttRobotAdapter::new("acme", "vda5050", 3);
    assert_eq!(adapter.manufacturer(), "acme");

    let order = vda5050_types::types::Order {
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
    let actions = vda5050_types::types::InstantActions {
        header_id: 1,
        timestamp: "2026-01-01T00:00:00.000Z".to_owned(),
        version: "3.0.0".to_owned(),
        manufacturer: "acme".to_owned(),
        serial_number: "AMR-001".to_owned(),
        actions: Vec::new(),
    };

    // Same adapter, two manufacturers: neither can be dispatched to, but the
    // refusal has to name the robot so the operator can tell which failed.
    for robot_id in ["acme:AMR-001", "sweepco:SCR-001"] {
        let err = adapter
            .dispatch(robot_id, &order)
            .await
            .expect_err("observe-only adapter refuses");
        assert!(
            matches!(
                err,
                AdapterError::UnknownRobot { .. } | AdapterError::Unsupported { .. }
            ),
            "{robot_id} must be refused, got {err:?}"
        );
        assert!(adapter.instant_action(robot_id, &actions).await.is_err());
    }
}

#[tokio::test]
async fn doors_are_visible_to_the_coordination_layer() {
    let adapter =
        MqttRobotAdapter::new("acme", "vda5050", 3).with_door_nodes(vec!["DOOR-IN".to_owned()]);
    let resources = adapter.shared_resources().await;
    assert_eq!(resources.len(), 1);
    assert_eq!(resources[0].kind, ResourceKind::Door);
    assert_eq!(resources[0].id, "DOOR-IN");
    // An empty contender list is honest before a route exists.
    assert!(resources[0].contenders.is_empty());
}

#[tokio::test]
async fn a_robot_on_a_door_node_is_reported_as_holding_it() {
    let adapter =
        MqttRobotAdapter::new("acme", "vda5050", 3).with_door_nodes(vec!["DOOR-IN".to_owned()]);
    adapter
        .observe_state(&state("acme", "AMR-001", "DOOR-IN", false))
        .await;
    adapter
        .observe_state(&state("acme", "AMR-002", "AS-1M", true))
        .await;

    assert!(adapter.holds("acme:AMR-001", "DOOR-IN").await);
    assert!(!adapter.holds("acme:AMR-002", "DOOR-IN").await);
    // A node that is not a declared door is never held.
    assert!(!adapter.holds("acme:AMR-002", "AS-1M").await);
}

#[tokio::test]
async fn two_adapters_do_not_leak_into_each_other() {
    // The gateway registers one adapter per manufacturer; a robot must only
    // ever be answerable by its own.
    let acme = MqttRobotAdapter::new("acme", "vda5050", 3);
    let sweepco = MqttRobotAdapter::new("sweepco", "vda5050", 3);
    acme.observe_state(&state("acme", "AMR-001", "AS-1M", false))
        .await;
    sweepco
        .observe_state(&state("sweepco", "SCR-001", "AS-1M", false))
        .await;

    assert_eq!(acme.robots().await.len(), 1);
    assert_eq!(sweepco.robots().await.len(), 1);
    assert_eq!(acme.robots().await[0].robot_id, "acme:AMR-001");
    assert_eq!(sweepco.robots().await[0].robot_id, "sweepco:SCR-001");
}
