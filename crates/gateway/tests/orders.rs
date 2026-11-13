//! Tests for mission issuing (`POST /api/orders`).
//!
//! The assembler is exercised directly: it needs no broker, and the interesting
//! behaviour — sequence-id parity, geometry lookup, rejection reasons — is all
//! in the transformation from an operator request to a wire-format order.

use serde_json::{json, Value};
use std::collections::BTreeMap;

use gateway::orders::{ActionSpec, OrderAssembler, OrderRequest, Waypoint};

/// The warehouse map the dashboard serves.
fn warehouse_map() -> Value {
    json!({
        "mapId": "plant-b/l1",
        "nodes": {
            "CONV-IN": { "x": 0.0, "y": 0.0, "theta": 0.0 },
            "AS-1W":   { "x": 0.0, "y": 6.0, "theta": 0.0 },
            "RK-AM":   { "x": 18.0, "y": 4.0, "theta": 0.0 },
            "RK-DM":   { "x": 18.0, "y": 16.0, "theta": 0.0 },
            "CONV-OUT":{ "x": 36.0, "y": 24.0, "theta": 0.0 },
            "CHG-01":  { "x": 0.0, "y": 24.0, "theta": 0.0 }
        },
        "edges": [
            { "from": "CONV-IN", "to": "AS-1W", "length": 6.0, "speed": 1.0 },
            { "from": "AS-1W", "to": "RK-AW", "length": 6.32, "speed": 0.6 },
            { "from": "RK-AW", "to": "RK-AM", "length": 12.0, "speed": 0.8 }
        ]
    })
}

fn request(nodes: Vec<Waypoint>) -> OrderRequest {
    OrderRequest {
        manufacturer: "acme".to_owned(),
        serial_number: "AMR-001".to_owned(),
        order_id: "order-ui-1".to_owned(),
        order_update_id: 0,
        order_description: None,
        nodes,
    }
}

fn waypoint(node_id: &str, actions: Vec<ActionSpec>) -> Waypoint {
    Waypoint {
        node_id: node_id.to_owned(),
        actions,
        released: None,
    }
}

fn pick(duration_s: f64) -> ActionSpec {
    ActionSpec {
        action_type: "pick".to_owned(),
        action_id: None,
        duration_s: Some(duration_s),
        blocking_type: None,
        parameters: BTreeMap::default(),
    }
}

#[test]
fn nodes_get_even_and_edges_odd_sequence_ids() {
    let map = warehouse_map();
    let assembler = OrderAssembler::new(&map);
    let order = assembler
        .assemble(&request(vec![
            waypoint("CONV-IN", vec![pick(3.0)]),
            waypoint("RK-AM", vec![pick(5.0)]),
            waypoint("CONV-OUT", vec![]),
        ]))
        .expect("a valid mission");

    let node_ids: Vec<u32> = order.nodes.iter().map(|n| n.sequence_id).collect();
    let edge_ids: Vec<u32> = order.edges.iter().map(|e| e.sequence_id).collect();
    assert_eq!(node_ids, vec![0, 2, 4], "nodes must be even");
    assert_eq!(edge_ids, vec![1, 3], "edges must be odd and in between");

    // The order must read node → edge → node → edge → node.
    assert_eq!(order.nodes[1].sequence_id, order.edges[0].sequence_id + 1);
    assert_eq!(order.edges[0].sequence_id + 1, order.nodes[1].sequence_id);
    assert_eq!(order.edges[0].start_node_id.as_deref(), Some("CONV-IN"));
    assert_eq!(order.edges[0].end_node_id.as_deref(), Some("RK-AM"));
}

#[test]
fn geometry_is_looked_up_from_the_served_map() {
    let map = warehouse_map();
    let assembler = OrderAssembler::new(&map);
    let order = assembler
        .assemble(&request(vec![
            waypoint("CONV-IN", vec![]),
            waypoint("AS-1W", vec![]),
        ]))
        .expect("a valid mission");

    let position = order.nodes[1]
        .node_position
        .as_ref()
        .expect("position taken from the map");
    assert_eq!(position.x, 0.0);
    assert_eq!(position.y, 6.0);
    assert_eq!(position.map_id, "plant-b/l1");
    assert!(position.localized);

    // CONV-IN → AS-1W is a real edge of the map, so its length is carried over.
    let length = order.edges[0].length.expect("known edge length");
    assert!((length - 6.0).abs() < 1e-9);
}

#[test]
fn header_ids_increase_and_are_integers() {
    let map = warehouse_map();
    let assembler = OrderAssembler::new(&map);
    let first = assembler
        .assemble(&request(vec![waypoint("CHG-01", vec![])]))
        .expect("valid");
    let second = assembler
        .assemble(&request(vec![waypoint("CHG-01", vec![])]))
        .expect("valid");
    assert!(second.header_id > first.header_id, "headerId must increase");
}

#[test]
fn actions_get_default_ids_and_a_duration_parameter() {
    let map = warehouse_map();
    let assembler = OrderAssembler::new(&map);
    let order = assembler
        .assemble(&request(vec![waypoint("RK-AM", vec![pick(8.0)])]))
        .expect("valid");

    let action = &order.nodes[0].actions[0];
    assert_eq!(action.action_type, "pick");
    assert_eq!(action.action_id, "a1", "actionId is generated");
    // HARD is the default: the robot must finish before driving on.
    assert_eq!(
        action.blocking_type,
        vda5050_types::types::BlockingType::HARD
    );
    let duration = action
        .action_parameters
        .iter()
        .find(|p| p.key == "duration")
        .expect("duration parameter");
    assert_eq!(duration.value, json!(8.0));
}

#[test]
fn an_unreleased_waypoint_makes_the_robot_wait() {
    let map = warehouse_map();
    let assembler = OrderAssembler::new(&map);
    let order = assembler
        .assemble(&request(vec![Waypoint {
            node_id: "RK-AM".to_owned(),
            actions: vec![],
            released: Some(false),
        }]))
        .expect("valid");
    assert!(!order.nodes[0].released);
}

#[test]
fn a_node_that_is_not_on_the_map_is_rejected_with_the_map_id() {
    let map = warehouse_map();
    let assembler = OrderAssembler::new(&map);
    let reasons = assembler
        .assemble(&request(vec![waypoint("NOWHERE", vec![])]))
        .expect_err("unknown node");

    assert!(
        reasons
            .iter()
            .any(|r| r.contains("NOWHERE") && r.contains("plant-b/l1")),
        "the operator needs to know which map and which node: {reasons:?}"
    );
}

#[test]
fn every_problem_is_reported_in_one_pass() {
    let map = warehouse_map();
    let assembler = OrderAssembler::new(&map);
    let mut broken = request(vec![waypoint("NOWHERE", vec![])]);
    broken.order_id = String::new();
    broken.manufacturer = "ac/me".to_owned();
    let reasons = assembler.assemble(&broken).expect_err("several problems");

    assert!(reasons.iter().any(|r| r.contains("orderId")));
    assert!(reasons.iter().any(|r| r.contains("manufacturer")));
    assert!(reasons.iter().any(|r| r.contains("NOWHERE")));
}

#[test]
fn a_mission_needs_at_least_one_waypoint() {
    let map = warehouse_map();
    let assembler = OrderAssembler::new(&map);
    let reasons = assembler
        .assemble(&request(vec![]))
        .expect_err("empty mission");
    assert!(reasons.iter().any(|r| r.contains("at least one waypoint")));
}

#[test]
fn duplicate_action_ids_are_rejected() {
    let map = warehouse_map();
    let assembler = OrderAssembler::new(&map);
    let reasons = assembler
        .assemble(&request(vec![waypoint(
            "RK-AM",
            vec![
                ActionSpec {
                    action_type: "pick".to_owned(),
                    action_id: Some("pk-1".to_owned()),
                    duration_s: Some(1.0),
                    blocking_type: None,
                    parameters: BTreeMap::default(),
                },
                ActionSpec {
                    action_type: "drop".to_owned(),
                    action_id: Some("pk-1".to_owned()),
                    duration_s: Some(1.0),
                    blocking_type: None,
                    parameters: BTreeMap::default(),
                },
            ],
        )]))
        .expect_err("duplicate actionId");
    assert!(reasons.iter().any(|r| r.contains("duplicate actionId")));
}

#[test]
fn a_negative_duration_is_rejected() {
    let map = warehouse_map();
    let assembler = OrderAssembler::new(&map);
    let reasons = assembler
        .assemble(&request(vec![waypoint("RK-AM", vec![pick(-1.0)])]))
        .expect_err("negative duration");
    assert!(reasons.iter().any(|r| r.contains("non-negative")));
}

#[test]
fn the_assembled_order_passes_the_official_schema() {
    // The dashboard never sends this shape; it must still be wire-valid, because
    // a robot that rejects it would only report `VALIDATION_FAILURE`.
    let map = warehouse_map();
    let assembler = OrderAssembler::new(&map);
    let order = assembler
        .assemble(&request(vec![
            waypoint("CONV-IN", vec![pick(3.0)]),
            waypoint("RK-AM", vec![pick(5.0)]),
            waypoint("RK-DM", vec![pick(2.0)]),
            waypoint("CONV-OUT", vec![]),
            waypoint("CHG-01", vec![]),
        ]))
        .expect("valid");

    let value = serde_json::to_value(&order).expect("serialises");
    let validator = vda5050_types::MessageValidator::new().expect("schemas compile");
    validator
        .validate(vda5050_types::Topic::Order, &value)
        .expect("the assembled order is valid VDA 5050");
}
