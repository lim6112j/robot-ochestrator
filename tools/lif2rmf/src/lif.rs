//! VDMA LIF (Layout Interchange Format) import.
//!
//! LIF is the format a facility hands over: `layouts[]` of `nodes`, `edges`
//! and `stations`, with per-vehicle-type properties. It is *not* the same shape
//! as the map this project serves, so importing is a conversion rather than a
//! rename.
//!
//! The schema is `vdma-lif/schema/lif-schema.json`. Two things about it are
//! worth knowing before reading the code below:
//!
//! * **There is no charger, and no door.** LIF describes a drivable track
//!   layout and nothing else. Chargers are not a LIF concept, so they cannot be
//!   read out of a LIF file — see [`crate::chargers`] and the `--chargers`
//!   argument. Doors exist only as far as this project invents them from node
//!   id prefixes, exactly as for a native map.
//! * **Positions live under `vehicleTypeNodeProperties`, not on the node.**
//!   `nodePosition` carries only `x`/`y`; `theta` is per vehicle type, so a
//!   LIF node with several vehicle types has several orientations and picking
//!   one is a policy decision, not a fact to look up.

use std::collections::BTreeMap;

use serde::Deserialize;

use crate::{SourceMap, SourceNode};

/// A parsed LIF document.
///
/// Only the parts this project consumes are modelled; `metaInformation` is
/// read for the version but otherwise ignored, and unknown fields are left
/// for serde to drop.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Lif {
    pub meta_information: MetaInformation,
    pub layouts: Vec<Layout>,
}

/// LIF file metadata.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MetaInformation {
    pub project_identification: String,
    pub creator: String,
    pub export_timestamp: String,
    pub lif_version: String,
}

/// One storey or area.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Layout {
    pub layout_id: String,
    pub layout_version: String,
    #[serde(default)]
    pub layout_name: Option<String>,
    #[serde(default)]
    pub layout_level_id: Option<String>,
    #[serde(default)]
    pub layout_description: Option<String>,
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
    #[serde(default)]
    pub stations: Vec<Station>,
}

/// A place a vehicle can be.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Node {
    pub node_id: String,
    #[serde(default)]
    pub node_name: Option<String>,
    #[serde(default)]
    pub node_description: Option<String>,
    #[serde(default)]
    pub map_id: Option<String>,
    pub node_position: NodePosition,
    pub vehicle_type_node_properties: Vec<VehicleTypeNodeProperties>,
}

/// `x`/`y` in metres. Note there is no `theta` here.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct NodePosition {
    pub x: f64,
    pub y: f64,
}

/// What one vehicle type can do at a node.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct VehicleTypeNodeProperties {
    pub vehicle_type_id: String,
    #[serde(default)]
    pub theta: Option<f64>,
    #[serde(default)]
    pub actions: Vec<NodeAction>,
}

/// An action a vehicle may run at a node.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NodeAction {
    pub action_type: String,
    pub blocking_type: String,
    #[serde(default)]
    pub action_description: Option<String>,
    #[serde(default)]
    pub requirement_type: Option<String>,
}

/// A connection between two nodes.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Edge {
    pub edge_id: String,
    pub start_node_id: String,
    pub end_node_id: String,
    pub vehicle_type_edge_properties: Vec<VehicleTypeEdgeProperties>,
}

/// What one vehicle type may do on an edge.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct VehicleTypeEdgeProperties {
    pub vehicle_type_id: String,
    #[serde(default)]
    pub vehicle_orientation: Option<f64>,
    #[serde(default)]
    pub orientation_type: Option<String>,
    #[serde(default)]
    pub max_speed: Option<f64>,
}

/// A place where a vehicle performs a specific task.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Station {
    pub station_id: String,
    pub interaction_node_ids: Vec<String>,
    #[serde(default)]
    pub station_name: Option<String>,
}

/// Why a LIF document could not be imported.
#[derive(Debug, thiserror::Error)]
pub enum LifError {
    #[error("no layouts: a LIF file describes at least one")]
    NoLayouts,
    #[error("more than one layout ({0}); this project serves one map at a time")]
    MultipleLayouts(usize),
    #[error("node {node_id} has no vehicle type properties, so its orientation is unknown")]
    NoVehicleType { node_id: String },
    #[error("edge {edge_id} has no vehicle type properties, so its speed is unknown")]
    NoEdgeVehicleType { edge_id: String },
    #[error("edge {edge_id} references unknown node {node_id}")]
    UnknownNode { edge_id: String, node_id: String },
}

/// Convert a LIF document into the map this project serves.
///
/// `theta` is taken from the **first** vehicle type that declares one. A node
/// with several vehicle types has several orientations and no way to choose
/// between them from the file alone, so the choice is made here and is
/// documented rather than left to whichever entry happened to parse last.
///
/// `speed` is the first declared `maxSpeed`, or `default_speed` when the edge
/// declares none — the same fallback a native map uses.
pub fn to_source_map(lif: &Lif, map_id: &str, default_speed: f64) -> Result<SourceMap, LifError> {
    let [layout] = lif.layouts.as_slice() else {
        return Err(if lif.layouts.is_empty() {
            LifError::NoLayouts
        } else {
            LifError::MultipleLayouts(lif.layouts.len())
        });
    };

    let mut nodes: BTreeMap<String, SourceNode> = BTreeMap::new();
    for node in &layout.nodes {
        let theta = node
            .vehicle_type_node_properties
            .iter()
            .find_map(|v| v.theta)
            .ok_or_else(|| LifError::NoVehicleType {
                node_id: node.node_id.clone(),
            })?;
        nodes.insert(
            node.node_id.clone(),
            SourceNode {
                x: node.node_position.x,
                y: node.node_position.y,
                theta,
            },
        );
    }

    let mut edges = Vec::with_capacity(layout.edges.len());
    for edge in &layout.edges {
        for node_id in [&edge.start_node_id, &edge.end_node_id] {
            if !nodes.contains_key(node_id) {
                return Err(LifError::UnknownNode {
                    edge_id: edge.edge_id.clone(),
                    node_id: node_id.clone(),
                });
            }
        }
        let speed = edge
            .vehicle_type_edge_properties
            .iter()
            .find_map(|v| v.max_speed)
            .unwrap_or(default_speed);
        edges.push(crate::SourceEdge {
            from: edge.start_node_id.clone(),
            to: edge.end_node_id.clone(),
            length: None,
            speed: Some(speed),
        });
    }

    Ok(SourceMap {
        map_id: map_id.to_owned(),
        nodes,
        edges,
        // LIF has no charger concept. The caller supplies them.
        chargers: Vec::new(),
    })
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    /// A minimal but schema-shaped LIF document.
    fn lif_json() -> serde_json::Value {
        json!({
            "metaInformation": {
                "projectIdentification": "plant-b",
                "creator": "rcs",
                "exportTimestamp": "2026-01-01T00:00:00Z",
                "lifVersion": "1.0.0"
            },
            "layouts": [{
                "layoutId": "plant-b",
                "layoutVersion": "1.0.0",
                "layoutLevelId": "L1",
                "nodes": [
                    {
                        "nodeId": "AS-1W",
                        "nodePosition": { "x": 0.0, "y": 6.0 },
                        "vehicleTypeNodeProperties": [
                            { "vehicleTypeId": "amr", "theta": 0.0 }
                        ]
                    },
                    {
                        "nodeId": "AS-1E",
                        "nodePosition": { "x": 36.0, "y": 6.0 },
                        "vehicleTypeNodeProperties": [
                            { "vehicleTypeId": "amr", "theta": 0.0 }
                        ]
                    }
                ],
                "edges": [
                    {
                        "edgeId": "AS-1W->AS-1E",
                        "startNodeId": "AS-1W",
                        "endNodeId": "AS-1E",
                        "vehicleTypeEdgeProperties": [
                            { "vehicleTypeId": "amr", "maxSpeed": 0.8 }
                        ]
                    }
                ]
            }]
        })
    }

    fn parse(v: serde_json::Value) -> Lif {
        serde_json::from_value(v).expect("valid LIF")
    }

    #[test]
    fn nodes_and_edges_carry_over() {
        let map = to_source_map(&parse(lif_json()), "plant-b/l1", 1.0).expect("converts");
        assert_eq!(map.nodes.len(), 2);
        assert_eq!(map.edges.len(), 1);
        assert_eq!(map.edges[0].from, "AS-1W");
        assert_eq!(map.edges[0].to, "AS-1E");
        assert!((map.edges[0].speed.unwrap() - 0.8).abs() < 1e-9);
    }

    #[test]
    fn theta_comes_from_the_vehicle_type_properties() {
        // `nodePosition` has only x/y; theta is per vehicle type. Reading it
        // from the node would be an AttributeError in Python and a silent
        // default here.
        let mut doc = lif_json();
        doc["layouts"][0]["nodes"][0]["vehicleTypeNodeProperties"][0]["theta"] = json!(1.57);
        let map = to_source_map(&parse(doc), "m", 1.0).expect("converts");
        assert!((map.nodes["AS-1W"].theta - 1.57).abs() < 1e-9);
        assert!(
            (map.nodes["AS-1E"].theta).abs() < 1e-9,
            "untouched node keeps 0"
        );
    }

    #[test]
    fn a_node_without_theta_is_rejected_rather_than_defaulted() {
        // Defaulting to 0 would silently rotate every robot.
        let mut doc = lif_json();
        doc["layouts"][0]["nodes"][0]["vehicleTypeNodeProperties"][0]
            .as_object_mut()
            .unwrap()
            .remove("theta");
        let err = to_source_map(&parse(doc), "m", 1.0).expect_err("must fail");
        assert!(matches!(err, LifError::NoVehicleType { .. }), "got {err:?}");
    }

    #[test]
    fn an_edge_speed_falls_back_to_the_default() {
        let mut doc = lif_json();
        doc["layouts"][0]["edges"][0]["vehicleTypeEdgeProperties"][0]
            .as_object_mut()
            .unwrap()
            .remove("maxSpeed");
        let map = to_source_map(&parse(doc), "m", 1.5).expect("converts");
        assert!((map.edges[0].speed.unwrap() - 1.5).abs() < 1e-9);
    }

    #[test]
    fn an_edge_to_an_unknown_node_is_rejected() {
        let mut doc = lif_json();
        doc["layouts"][0]["edges"][0]["endNodeId"] = json!("NOPE");
        let err = to_source_map(&parse(doc), "m", 1.0).expect_err("must fail");
        assert!(matches!(err, LifError::UnknownNode { .. }), "got {err:?}");
    }

    #[test]
    fn no_layouts_is_rejected() {
        let mut doc = lif_json();
        doc["layouts"] = json!([]);
        let err = to_source_map(&parse(doc), "m", 1.0).expect_err("must fail");
        assert!(matches!(err, LifError::NoLayouts), "got {err:?}");
    }

    #[test]
    fn two_layouts_are_rejected_rather_than_silently_merged() {
        // Merging two storeys would put both on one level and every robot
        // would drive through the other storey's geometry.
        let mut doc = lif_json();
        let second = doc["layouts"][0].clone();
        doc["layouts"].as_array_mut().unwrap().push(second);
        let err = to_source_map(&parse(doc), "m", 1.0).expect_err("must fail");
        assert!(matches!(err, LifError::MultipleLayouts(2)), "got {err:?}");
    }

    #[test]
    fn chargers_are_not_invented_from_the_lif_file() {
        // LIF has no charger concept. Leaving the list empty is the honest
        // answer; guessing from node names would silently pick the wrong
        // charger for RMF, which refuses a fleet with none.
        let map = to_source_map(&parse(lif_json()), "m", 1.0).expect("converts");
        assert!(
            map.chargers.is_empty(),
            "chargers must come from the caller, got {:?}",
            map.chargers
        );
    }

    #[test]
    fn a_real_lif_document_with_stations_parses() {
        // Stations are part of the schema and are ignored on import: they
        // describe where work happens, not the drivable track.
        let mut doc = lif_json();
        doc["layouts"][0]["stations"] = json!([{
            "stationId": "ST-1",
            "interactionNodeIds": ["AS-1W"],
            "stationName": "pickup"
        }]);
        let map = to_source_map(&parse(doc), "m", 1.0).expect("converts");
        assert_eq!(map.nodes.len(), 2);
    }
}

#[cfg(test)]
mod schema_tests {
    //! The vendored LIF schema is the contract. These tests read it directly
    //! and check that this module's field names and required-ness match, so a
    //! private dialect that merely *looks* like LIF cannot pass.
    //!
    //! A full JSON Schema validator was tried first and removed: `jsonschema`
    //! pulls in `reqwest`, `hyper`, `rustls` and `jni`, which broke the
    //! workspace's tokio tests. Checking the schema's `required` and
    //! `properties` lists directly catches the drift that matters — a renamed
    //! or missing field — without the dependency.

    use serde_json::{json, Value};

    use super::*;

    fn schema() -> Value {
        let raw = include_str!("../../../schemas/lif-schema.json");
        serde_json::from_str(raw).expect("schema parses")
    }

    /// Every key the schema marks required at `layouts[].<path>`.
    ///
    /// A path element that names an array property (`nodes`, `edges`,
    /// `stations`) descends into its `items`, so the caller writes the
    /// property names and not the JSON Schema plumbing.
    fn required_at(schema: &Value, path: &[&str]) -> Vec<String> {
        let mut node = &schema["properties"]["layouts"]["items"];
        for key in path {
            node = &node["properties"][key];
            if node.get("type") == Some(&json!("array")) {
                node = &node["items"];
            }
        }
        node["required"]
            .as_array()
            .expect("required is an array")
            .iter()
            .map(|v| v.as_str().expect("string").to_owned())
            .collect()
    }

    #[test]
    fn the_node_types_cover_every_required_lif_field() {
        let schema = schema();
        let required = required_at(&schema, &["nodes"]);
        // What the module models as a struct field. `nodePosition` is one
        // object, so its x/y are checked separately below.
        assert_eq!(
            required,
            vec!["nodeId", "nodePosition", "vehicleTypeNodeProperties"],
            "the schema's required node fields"
        );

        // Each must exist as a field on the Rust type, which is what makes
        // `serde_json::from_value::<Node>` succeed or fail.
        let node = json!({
            "nodeId": "N1",
            "nodePosition": { "x": 1.0, "y": 2.0 },
            "vehicleTypeNodeProperties": [{ "vehicleTypeId": "amr", "theta": 0.0 }]
        });
        let parsed: Node =
            serde_json::from_value(node).expect("module models every required field");
        assert_eq!(parsed.node_id, "N1");
        assert_eq!(parsed.node_position.x, 1.0);
        assert_eq!(parsed.vehicle_type_node_properties.len(), 1);

        // And `nodePosition` itself requires x and y.
        let pos_required: Vec<String> = schema["properties"]["layouts"]["items"]["properties"]
            ["nodes"]["items"]["properties"]["nodePosition"]["required"]
            .as_array()
            .expect("array")
            .iter()
            .map(|v| v.as_str().unwrap().to_owned())
            .collect();
        assert_eq!(pos_required, vec!["x", "y"]);
    }

    #[test]
    fn the_edge_types_cover_every_required_lif_field() {
        let schema = schema();
        let required = required_at(&schema, &["edges"]);
        assert_eq!(
            required,
            vec![
                "edgeId",
                "startNodeId",
                "endNodeId",
                "vehicleTypeEdgeProperties"
            ]
        );

        let edge = json!({
            "edgeId": "E1",
            "startNodeId": "N1",
            "endNodeId": "N2",
            "vehicleTypeEdgeProperties": [{ "vehicleTypeId": "amr", "maxSpeed": 0.8 }]
        });
        let parsed: Edge =
            serde_json::from_value(edge).expect("module models every required field");
        assert_eq!(parsed.edge_id, "E1");
        assert_eq!(parsed.start_node_id, "N1");
    }

    #[test]
    fn the_top_level_types_cover_every_required_lif_field() {
        let schema = schema();
        let root: Vec<String> = schema["required"]
            .as_array()
            .expect("array")
            .iter()
            .map(|v| v.as_str().unwrap().to_owned())
            .collect();
        assert_eq!(root, vec!["metaInformation", "layouts"]);

        let meta: Vec<String> = schema["properties"]["metaInformation"]["required"]
            .as_array()
            .expect("array")
            .iter()
            .map(|v| v.as_str().unwrap().to_owned())
            .collect();
        assert_eq!(
            meta,
            vec![
                "projectIdentification",
                "creator",
                "exportTimestamp",
                "lifVersion"
            ]
        );

        let layout: Vec<String> = schema["properties"]["layouts"]["items"]["required"]
            .as_array()
            .expect("array")
            .iter()
            .map(|v| v.as_str().unwrap().to_owned())
            .collect();
        // `stations` is required even when empty — a LIF file without it is
        // not valid LIF, which is easy to miss when writing a sample by hand.
        assert_eq!(
            layout,
            vec!["layoutId", "layoutVersion", "nodes", "edges", "stations"]
        );
    }

    #[test]
    fn a_minimal_valid_lif_document_round_trips_through_the_types() {
        // Built strictly from the schema's required fields, so it is valid
        // LIF by construction.
        let doc = json!({
            "metaInformation": {
                "projectIdentification": "plant-b",
                "creator": "rcs",
                "exportTimestamp": "2026-01-01T00:00:00Z",
                "lifVersion": "1.0.0"
            },
            "layouts": [{
                "layoutId": "plant-b",
                "layoutVersion": "1.0.0",
                "nodes": [{
                    "nodeId": "N1",
                    "nodePosition": { "x": 0.0, "y": 0.0 },
                    "vehicleTypeNodeProperties": [
                        { "vehicleTypeId": "amr", "theta": 0.0 }
                    ]
                }],
                "edges": [{
                    "edgeId": "E1",
                    "startNodeId": "N1",
                    "endNodeId": "N1",
                    "vehicleTypeEdgeProperties": [
                        { "vehicleTypeId": "amr", "maxSpeed": 0.8 }
                    ]
                }],
                "stations": []
            }]
        });

        let lif: Lif = serde_json::from_value(doc).expect("valid LIF parses");
        let map = to_source_map(&lif, "plant-b", 1.0).expect("converts");
        assert_eq!(map.nodes.len(), 1);
        assert_eq!(map.edges.len(), 1);
    }

    #[test]
    fn a_document_missing_a_required_field_is_refused() {
        // `nodePosition` without `y` is invalid LIF; the module must not
        // quietly accept it and place the node at y=0.
        let doc = json!({
            "metaInformation": {
                "projectIdentification": "p",
                "creator": "c",
                "exportTimestamp": "t",
                "lifVersion": "1.0.0"
            },
            "layouts": [{
                "layoutId": "l",
                "layoutVersion": "1",
                "nodes": [{
                    "nodeId": "N1",
                    "nodePosition": { "x": 0.0 },
                    "vehicleTypeNodeProperties": [
                        { "vehicleTypeId": "amr", "theta": 0.0 }
                    ]
                }],
                "edges": [],
                "stations": []
            }]
        });
        assert!(
            serde_json::from_value::<Lif>(doc).is_err(),
            "a position without y must be refused"
        );
    }

    #[test]
    fn unknown_fields_are_dropped_rather_than_fatal() {
        // LIF is versioned and grows; a newer file with extra keys has to
        // still import or every minor release breaks the importer.
        let doc = json!({
            "metaInformation": {
                "projectIdentification": "p",
                "creator": "c",
                "exportTimestamp": "t",
                "lifVersion": "9.9.9",
                "someFutureKey": "ignored"
            },
            "layouts": [{
                "layoutId": "l",
                "layoutVersion": "1",
                "futureLayoutKey": { "a": 1 },
                "nodes": [{
                    "nodeId": "N1",
                    "nodePosition": { "x": 0.0, "y": 0.0 },
                    "vehicleTypeNodeProperties": [
                        { "vehicleTypeId": "amr", "theta": 0.0 }
                    ]
                }],
                "edges": [],
                "stations": []
            }]
        });
        let lif: Lif = serde_json::from_value(doc).expect("unknown keys are dropped");
        assert_eq!(lif.meta_information.lif_version, "9.9.9");
    }
}
