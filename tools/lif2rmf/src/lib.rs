//! Convert a VDA 5050 map (node/edge JSON) into an RMF building map.
//!
//! The two formats describe the same building at different levels of detail:
//!
//! | VDA 5050 map | RMF building map |
//! |---|---|
//! | `nodes` | level `vertices` |
//! | `edges` | level `lanes` |
//! | (none) | level `doors` — shared resources |
//!
//! Doing this by hand drifts the moment the warehouse map changes, so the
//! conversion is code and is covered by tests.
//!
//! Two RMF details are easy to get wrong and are handled deliberately:
//!
//! * A door is a **segment**, and a lane only inherits `door_name` when it
//!   actually intersects that segment. `segments_intersect` rejects collinear
//!   segments, so a door placed *along* a corridor is invisible to RMF. Doors
//!   are therefore emitted perpendicular to the corridor they guard.
//! * Only vertices referenced by a lane enter the nav graph, so the extra
//!   vertices a door needs do not pollute it.

use std::collections::{BTreeMap, BTreeSet};

use serde::Deserialize;

/// The VDA 5050 map format used by the simulator and the dashboard.
#[derive(Debug, Clone, Deserialize)]
pub struct SourceMap {
    #[serde(rename = "mapId")]
    pub map_id: String,
    /// Node id → position.
    pub nodes: BTreeMap<String, SourceNode>,
    pub edges: Vec<SourceEdge>,
}

/// A node position.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct SourceNode {
    pub x: f64,
    pub y: f64,
    #[serde(default)]
    pub theta: f64,
}

/// A directed edge. The warehouse map emits both directions, so the same
/// pair usually appears twice.
#[derive(Debug, Clone, Deserialize)]
pub struct SourceEdge {
    pub from: String,
    pub to: String,
    #[serde(default)]
    pub length: Option<f64>,
    #[serde(default)]
    pub speed: Option<f64>,
}

/// Conversion options.
#[derive(Debug, Clone)]
pub struct Options {
    /// Level name; the warehouse is single-storey.
    pub level: String,
    /// Node ids starting with this prefix become RMF doors.
    pub door_prefix: String,
    /// Half-width of the door segment, in metres.
    ///
    /// Wide enough that lanes passing through the node cross it, narrow
    /// enough not to look like a wall.
    pub door_width: f64,
    /// Default speed for lanes whose edge carries none.
    pub default_speed: f64,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            level: "L1".to_owned(),
            door_prefix: "DOOR".to_owned(),
            door_width: 0.5,
            default_speed: 1.0,
        }
    }
}

/// The RMF building map, in the tuple notation the YAML uses.
#[derive(Debug, Clone, PartialEq)]
pub struct BuildingMap {
    pub name: String,
    pub coordinate_system: String,
    pub levels: Vec<Level>,
}

/// One storey.
#[derive(Debug, Clone, PartialEq)]
pub struct Level {
    pub name: String,
    pub elevation: f64,
    /// `[x, y, z, name, params]`.
    pub vertices: Vec<Vertex>,
    /// `[start_idx, end_idx, params]`.
    pub lanes: Vec<Lane>,
    /// `[v1_idx, v2_idx, params]`.
    pub doors: Vec<Door>,
}

/// A level vertex.
#[derive(Debug, Clone, PartialEq)]
pub struct Vertex {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub name: String,
}

/// A navigable lane.
///
/// `is_bidirectional` is emitted as the YAML key `bidirectional`: that is the
/// name `Edge.is_bidirectional()` looks up, and a lane without it defaults to
/// one-way — which would strand every robot at the far end of the warehouse.
#[derive(Debug, Clone, PartialEq)]
pub struct Lane {
    pub start_idx: usize,
    pub end_idx: usize,
    pub graph_idx: i64,
    pub is_bidirectional: bool,
    pub speed_limit: f64,
    /// Heading the robot should hold, in radians, when it has no choice.
    pub orientation: Option<f64>,
}

/// A door: a shared resource a robot must reserve.
#[derive(Debug, Clone, PartialEq)]
pub struct Door {
    pub v1_idx: usize,
    pub v2_idx: usize,
    pub name: String,
    /// RMF door type, e.g. `double_sliding`.
    pub door_type: String,
}

/// What went wrong while converting.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("edge references unknown node '{node}'")]
    UnknownNode { node: String },
    #[error("door node '{node}' has no neighbour to orient against")]
    DoorWithoutNeighbour { node: String },
}

/// Convert a VDA 5050 map into an RMF building map.
///
/// # Errors
/// Returns [`Error::UnknownNode`] when an edge references a node the map does
/// not define, and [`Error::DoorWithoutNeighbour`] when a door node is not
/// connected to anything (a door with no corridor cannot be oriented).
pub fn convert(map: &SourceMap, options: &Options) -> Result<BuildingMap, Error> {
    for edge in &map.edges {
        for endpoint in [&edge.from, &edge.to] {
            if !map.nodes.contains_key(endpoint) {
                return Err(Error::UnknownNode {
                    node: endpoint.clone(),
                });
            }
        }
    }

    // Doors first: they add vertices of their own, so lane indices must be
    // computed after every vertex exists.
    let door_nodes: Vec<&String> = map
        .nodes
        .keys()
        .filter(|id| id.starts_with(&options.door_prefix))
        .collect();

    let mut vertices: Vec<Vertex> = Vec::new();
    let mut index_of: BTreeMap<&str, usize> = BTreeMap::new();
    for (id, node) in &map.nodes {
        index_of.insert(id.as_str(), vertices.len());
        vertices.push(Vertex {
            x: node.x,
            y: node.y,
            z: 0.0,
            name: id.clone(),
        });
    }

    let mut doors = Vec::new();
    for door_node in door_nodes {
        let Some(centre) = map.nodes.get(door_node) else {
            continue;
        };
        let Some(axis) = corridor_axis(map, door_node) else {
            return Err(Error::DoorWithoutNeighbour {
                node: door_node.clone(),
            });
        };
        // Perpendicular to the corridor: a collinear door segment is ignored by
        // RMF's intersection test, which would silently drop the reservation.
        let (dx, dy) = (-axis.1, axis.0);
        let half = options.door_width / 2.0;
        let a = vertices.len();
        vertices.push(Vertex {
            x: centre.x - dx * half,
            y: centre.y - dy * half,
            z: 0.0,
            name: format!("{door_node}-door-a"),
        });
        let b = vertices.len();
        vertices.push(Vertex {
            x: centre.x + dx * half,
            y: centre.y + dy * half,
            z: 0.0,
            name: format!("{door_node}-door-b"),
        });
        doors.push(Door {
            v1_idx: a,
            v2_idx: b,
            name: door_node.clone(),
            door_type: "double_sliding".to_owned(),
        });
    }

    // The warehouse map emits both directions of every edge. RMF lanes are
    // unidirectional internally, but `is_bidirectional` on one lane per pair
    // keeps the nav graph from doubling.
    let mut seen: BTreeSet<(String, String)> = BTreeSet::new();
    let mut lanes = Vec::new();
    for edge in &map.edges {
        let pair = ordered_pair(&edge.from, &edge.to);
        if !seen.insert(pair.clone()) {
            continue;
        }
        let (Some(&start), Some(&end)) = (
            index_of.get(edge.from.as_str()),
            index_of.get(edge.to.as_str()),
        ) else {
            continue;
        };
        lanes.push(Lane {
            start_idx: start,
            end_idx: end,
            graph_idx: 0,
            is_bidirectional: true,
            speed_limit: edge.speed.unwrap_or(options.default_speed),
            // Keep the heading the map declares, so a robot arrives facing the
            // way the warehouse expects.
            orientation: map.nodes.get(&edge.to).map(|n| n.theta),
        });
    }

    Ok(BuildingMap {
        name: map.map_id.clone(),
        coordinate_system: "reference_image".to_owned(),
        levels: vec![Level {
            name: options.level.clone(),
            elevation: 0.0,
            vertices,
            lanes,
            doors,
        }],
    })
}

/// Order a node pair so `(a, b)` and `(b, a)` compare equal.
fn ordered_pair(a: &str, b: &str) -> (String, String) {
    if a <= b {
        (a.to_owned(), b.to_owned())
    } else {
        (b.to_owned(), a.to_owned())
    }
}

/// Direction of the longest edge at a node, normalised.
///
/// Used to orient a door perpendicular to the corridor it guards.
fn corridor_axis(map: &SourceMap, node: &str) -> Option<(f64, f64)> {
    let origin = map.nodes.get(node)?;
    let mut best: Option<(f64, f64, f64)> = None;
    for edge in &map.edges {
        let other = if edge.from == node {
            Some(&edge.to)
        } else if edge.to == node {
            Some(&edge.from)
        } else {
            None
        };
        let Some(other) = other else { continue };
        let Some(target) = map.nodes.get(other) else {
            continue;
        };
        let (dx, dy) = (target.x - origin.x, target.y - origin.y);
        let length = dx.hypot(dy);
        if length < 1e-9 {
            continue;
        }
        if best.is_none_or(|(_, _, best_len)| length > best_len) {
            best = Some((dx / length, dy / length, length));
        }
    }
    best.map(|(x, y, _)| (x, y))
}

/// Render the building map as RMF's YAML tuple notation.
///
/// Values are emitted as `[type_code, value]`, where 1 is string, 2 int,
/// 3 double and 4 bool — the encoding `ParamValue` expects.
#[must_use]
pub fn to_yaml(map: &BuildingMap) -> String {
    use std::fmt::Write;

    let mut out = String::new();
    let _ = writeln!(out, "name: {}", yaml_string(&map.name));
    let _ = writeln!(
        out,
        "coordinate_system: {}",
        yaml_string(&map.coordinate_system)
    );
    out.push_str("levels:\n");
    for level in &map.levels {
        let _ = writeln!(out, "  {}:", yaml_string(&level.name));
        let _ = writeln!(out, "    elevation: {}", level.elevation);
        out.push_str("    vertices:\n");
        for vertex in &level.vertices {
            let _ = writeln!(
                out,
                "      - [{}, {}, {}, {}, {{}}]",
                yaml_number(vertex.x),
                yaml_number(vertex.y),
                yaml_number(vertex.z),
                yaml_string(&vertex.name),
            );
        }
        out.push_str("    lanes:\n");
        for lane in &level.lanes {
            // `bidirectional` is the key RMF reads; `is_bidirectional` is
            // ignored and defaults the lane to one-way.
            let mut params = format!(
                "{{graph_idx: [2, {}], bidirectional: [4, {}], speed_limit: [3, {}]",
                lane.graph_idx,
                lane.is_bidirectional,
                yaml_number(lane.speed_limit)
            );
            if let Some(orientation) = lane.orientation {
                let _ = write!(params, ", orientation: [3, {}]", yaml_number(orientation));
            }
            params.push('}');
            let _ = writeln!(
                out,
                "      - [{}, {}, {}]",
                lane.start_idx, lane.end_idx, params
            );
        }
        out.push_str("    doors:\n");
        for door in &level.doors {
            let _ = writeln!(
                out,
                "      - [{}, {}, {{name: [1, {}], type: [1, {}], motion_axis: [1, start], motion_degrees: [3, 90], motion_direction: [2, 1]}}]",
                door.v1_idx,
                door.v2_idx,
                yaml_string(&door.name),
                yaml_string(&door.door_type),
            );
        }
        out.push_str("    walls: []\n");
        out.push_str("    floors: []\n");
        out.push_str("    measurements: []\n");
        out.push_str("    models: []\n");
        out.push_str("    layers:\n      {}\n");
        out.push_str("    x_meters: 1.0\n");
        out.push_str("    y_meters: 1.0\n");
        out.push_str("    flattened_x_offset: 0\n");
        out.push_str("    flattened_y_offset: 0\n");
    }
    out.push_str("lifts:\n  {}\n");
    out
}

/// Quote a YAML string only when it needs it.
fn yaml_string(value: &str) -> String {
    if value.is_empty()
        || value
            .chars()
            .any(|c| !matches!(c, 'a'..='z' | 'A'..='Z' | '0'..='9' | '-' | '_' | '.' | '/'))
    {
        format!("'{}'", value.replace('\'', "''"))
    } else {
        value.to_owned()
    }
}

/// Render a float without a trailing `.0`, which YAML would read as a string.
fn yaml_number(value: f64) -> String {
    // `as i64` would silently truncate a non-finite or out-of-range value, and
    // a coordinate that survives as a truncated integer is worse than an error.
    if value.is_finite() && value.fract() == 0.0 && value.abs() < 9.0e15 {
        // Already an exact integer, so the rounded form carries no precision.
        format!("{value:.0}")
    } else {
        format!("{value}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn small_map() -> SourceMap {
        serde_json::from_value(serde_json::json!({
            "mapId": "plant-b/l1",
            "nodes": {
                "CONV-IN":  { "x": 0.0,  "y": 0.0,  "theta": 0.0 },
                "AS-1W":    { "x": 0.0,  "y": 6.0,  "theta": 0.0 },
                "RK-AW":    { "x": 6.0,  "y": 4.0,  "theta": 0.0 },
                "RK-AM":    { "x": 18.0, "y": 4.0,  "theta": 0.0 },
                "DOOR-IN":  { "x": 12.0, "y": 0.0,  "theta": 0.0 }
            },
            "edges": [
                { "from": "CONV-IN", "to": "AS-1W", "length": 6.0, "speed": 1.0 },
                { "from": "AS-1W",   "to": "RK-AW", "length": 6.32, "speed": 0.6 },
                { "from": "RK-AW",   "to": "RK-AM", "length": 12.0, "speed": 0.8 },
                { "from": "CONV-IN", "to": "DOOR-IN", "length": 12.0, "speed": 0.8 },
                { "from": "DOOR-IN", "to": "RK-AM", "length": 12.0, "speed": 0.8 }
            ]
        }))
        .expect("valid map")
    }

    #[test]
    fn every_node_becomes_a_vertex() {
        let map = small_map();
        let out = convert(&map, &Options::default()).expect("converts");
        let level = &out.levels[0];
        assert_eq!(
            level.vertices.len(),
            map.nodes.len() + 2,
            "2 extra for the door"
        );
        let names: Vec<&str> = level.vertices.iter().map(|v| v.name.as_str()).collect();
        for id in map.nodes.keys() {
            assert!(names.contains(&id.as_str()), "{id} must be a vertex");
        }
    }

    #[test]
    fn a_pair_of_opposite_edges_becomes_one_lane() {
        // The warehouse map emits both directions; RMF should not see a
        // double lane or the nav graph doubles.
        let map = small_map();
        let distinct_pairs = convert(&map, &Options::default()).expect("converts").levels[0]
            .lanes
            .len();
        assert_eq!(distinct_pairs, 5, "five unordered pairs in the source");

        // Adding the reverse of an existing edge must not add a lane.
        let mut doubled = map.clone();
        doubled.edges.push(
            serde_json::from_value(serde_json::json!({
                "from": "AS-1W", "to": "CONV-IN", "length": 6.0, "speed": 1.0
            }))
            .unwrap(),
        );
        let out = convert(&doubled, &Options::default()).expect("converts");
        assert_eq!(
            out.levels[0].lanes.len(),
            distinct_pairs,
            "a reversed duplicate must not double the lane"
        );
        assert!(out.levels[0].lanes.iter().all(|l| l.is_bidirectional));
    }

    #[test]
    fn lane_speed_comes_from_the_edge() {
        let map = small_map();
        let out = convert(&map, &Options::default()).expect("converts");
        let lane = out.levels[0]
            .lanes
            .iter()
            .find(|l| out.levels[0].vertices[l.start_idx].name == "AS-1W")
            .expect("lane from AS-1W");
        assert!((lane.speed_limit - 0.6).abs() < 1e-9);
    }

    #[test]
    fn a_missing_speed_falls_back_to_the_default() {
        let mut map = small_map();
        for edge in &mut map.edges {
            edge.speed = None;
        }
        let out = convert(&map, &Options::default()).expect("converts");
        assert!(out.levels[0]
            .lanes
            .iter()
            .all(|l| (l.speed_limit - 1.0).abs() < 1e-9));
    }

    #[test]
    fn a_door_node_becomes_a_door_with_two_extra_vertices() {
        let map = small_map();
        let out = convert(&map, &Options::default()).expect("converts");
        let level = &out.levels[0];
        assert_eq!(level.doors.len(), 1);
        let door = &level.doors[0];
        assert_eq!(door.name, "DOOR-IN");
        assert_eq!(door.door_type, "double_sliding");
        assert!(door.v1_idx < level.vertices.len());
        assert!(door.v2_idx < level.vertices.len());
        assert_ne!(door.v1_idx, door.v2_idx);
    }

    #[test]
    fn the_door_segment_is_perpendicular_to_the_corridor() {
        // Collinear doors are invisible to RMF: `segments_intersect` rejects
        // them, so the reservation would silently never be requested.
        let map = small_map();
        let out = convert(&map, &Options::default()).expect("converts");
        let level = &out.levels[0];
        let door = &level.doors[0];
        let a = &level.vertices[door.v1_idx];
        let b = &level.vertices[door.v2_idx];
        let door_vec = (b.x - a.x, b.y - a.y);

        // The corridor at DOOR-IN runs to CONV-IN, i.e. along -x.
        let conv = level
            .vertices
            .iter()
            .find(|v| v.name == "CONV-IN")
            .expect("CONV-IN vertex");
        let corridor = (conv.x - 12.0, conv.y - 0.0);
        let dot = door_vec.0 * corridor.0 + door_vec.1 * corridor.1;
        assert!(
            dot.abs() < 1e-9,
            "door must be perpendicular, dot was {dot}"
        );
        // And it must actually straddle the node, not sit beside it.
        let mid = (f64::midpoint(a.x, b.x), f64::midpoint(a.y, b.y));
        assert!((mid.0 - 12.0).abs() < 1e-9 && mid.1.abs() < 1e-9);
    }

    #[test]
    fn an_edge_to_an_unknown_node_is_rejected() {
        let mut map = small_map();
        map.edges[0].to = "NOWHERE".to_owned();
        let err = convert(&map, &Options::default()).expect_err("unknown node");
        assert!(matches!(err, Error::UnknownNode { .. }));
        assert!(err.to_string().contains("NOWHERE"));
    }

    #[test]
    fn an_isolated_door_node_is_rejected() {
        // A door with no corridor cannot be oriented, and silently guessing
        // would produce a door no lane ever crosses.
        let mut map = small_map();
        map.edges
            .retain(|e| e.from != "DOOR-IN" && e.to != "DOOR-IN");
        let err = convert(&map, &Options::default()).expect_err("isolated door");
        assert!(matches!(err, Error::DoorWithoutNeighbour { .. }));
    }

    #[test]
    fn the_door_prefix_is_configurable() {
        let map = small_map();
        let out = convert(
            &map,
            &Options {
                door_prefix: "NOPE".to_owned(),
                ..Options::default()
            },
        )
        .expect("converts");
        assert!(out.levels[0].doors.is_empty(), "no node matched the prefix");
        assert_eq!(out.levels[0].vertices.len(), map.nodes.len());
    }

    #[test]
    fn the_yaml_is_parseable_and_keeps_the_structure() {
        let map = small_map();
        let out = convert(&map, &Options::default()).expect("converts");
        let yaml = to_yaml(&out);
        let parsed: serde_yaml::Value = serde_yaml::from_str(&yaml).expect("emitted YAML parses");
        let level = &parsed["levels"]["L1"];
        assert_eq!(
            level["vertices"].as_sequence().map(Vec::len),
            Some(out.levels[0].vertices.len())
        );
        assert_eq!(
            level["lanes"].as_sequence().map(Vec::len),
            Some(out.levels[0].lanes.len())
        );
        assert_eq!(level["doors"].as_sequence().map(Vec::len), Some(1));
        assert_eq!(parsed["name"].as_str(), Some("plant-b/l1"));
    }

    #[test]
    fn the_bidirectional_key_is_the_one_rmf_reads() {
        // RMF looks up `bidirectional`; `is_bidirectional` is silently ignored
        // and defaults the lane to one-way.
        let map = small_map();
        let out = convert(&map, &Options::default()).expect("converts");
        let yaml = to_yaml(&out);
        // Exact key, not a substring: `is_bidirectional` would also satisfy a
        // `contains("bidirectional")` check while RMF still ignored it.
        assert!(yaml.contains(", bidirectional: [4, true]"), "{yaml}");
        assert!(
            !yaml.contains("is_bidirectional"),
            "a key RMF does not read would strand every robot"
        );
    }

    #[test]
    fn floats_are_emitted_as_yaml_numbers_not_strings() {
        // `18.0` would parse as a string and break every consumer.
        assert_eq!(yaml_number(18.0), "18");
        assert_eq!(yaml_number(0.6), "0.6");
        assert_eq!(yaml_number(-3.0), "-3");
    }

    #[test]
    fn strings_are_quoted_only_when_needed() {
        assert_eq!(yaml_string("RK-AM"), "RK-AM");
        assert_eq!(yaml_string("plant-b/l1"), "plant-b/l1");
        assert_eq!(yaml_string(""), "''");
        assert_eq!(yaml_string("a b"), "'a b'");
    }
}
