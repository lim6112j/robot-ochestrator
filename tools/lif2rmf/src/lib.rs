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
//! * There is no bidirectional lane. The nav-graph parser adds one lane per
//!   entry and never looks at a `bidirectional` key, so both directions of an
//!   edge are emitted separately. `graph_idx` and `bidirectional` are
//!   `building_map` keys that this parser ignores.

use std::collections::BTreeMap;

use serde::Deserialize;

/// The VDA 5050 map format used by the simulator and the dashboard.
#[derive(Debug, Clone, Deserialize)]
pub struct SourceMap {
    #[serde(rename = "mapId")]
    pub map_id: String,
    /// Node id → position.
    pub nodes: BTreeMap<String, SourceNode>,
    pub edges: Vec<SourceEdge>,
    /// Node ids that are charging waypoints. RMF refuses to add a robot to a
    /// fleet whose nav graph has none, so a map without them cannot be used
    /// with the fleet adapter at all.
    #[serde(default)]
    pub chargers: Vec<String>,
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
    /// Emitted as the YAML key `is_charger`, which is what
    /// `parse_graph.cpp` reads to call `set_charger(true)`.
    pub is_charger: bool,
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
    pub speed_limit: f64,
    /// Heading the robot should hold, in radians, when it has no choice.
    pub orientation: Option<f64>,
    /// Name of the door this lane crosses, if any. RMF only reserves a door
    /// for a robot that is actually travelling through it.
    pub door_name: Option<String>,
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
            is_charger: map.chargers.iter().any(|c| c == id),
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
            is_charger: false,
        });
        let b = vertices.len();
        vertices.push(Vertex {
            x: centre.x + dx * half,
            y: centre.y + dy * half,
            z: 0.0,
            name: format!("{door_node}-door-b"),
            is_charger: false,
        });
        doors.push(Door {
            v1_idx: a,
            v2_idx: b,
            name: door_node.clone(),
            door_type: "double_sliding".to_owned(),
        });
    }

    // The warehouse map emits both directions of every edge, and the nav-graph
    // parser keeps every lane it is given: `parse_graph.cpp` calls
    // `graph.add_lane(begin, end)` once per entry and has no notion of a
    // bidirectional lane. Collapsing a pair into one lane would therefore leave
    // the reverse direction with no lane at all, stranding every robot that
    // drove somewhere. So the directions stay separate here.
    let mut lanes = Vec::new();
    for edge in &map.edges {
        let (Some(&start), Some(&end)) = (
            index_of.get(edge.from.as_str()),
            index_of.get(edge.to.as_str()),
        ) else {
            continue;
        };
        lanes.push(Lane {
            start_idx: start,
            end_idx: end,
            speed_limit: edge.speed.unwrap_or(options.default_speed),
            // Keep the heading the map declares, so a robot arrives facing the
            // way the warehouse expects.
            orientation: map.nodes.get(&edge.to).map(|n| n.theta),
            door_name: doors.iter().find_map(|door| {
                let a = &vertices[door.v1_idx];
                let b = &vertices[door.v2_idx];
                let from = &vertices[start];
                let to = &vertices[end];
                segments_intersect((a.x, a.y), (b.x, b.y), (from.x, from.y), (to.x, to.y))
                    .then(|| door.name.clone())
            }),
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

/// Whether two line segments cross, in the way RMF's own `segments_intersect`
/// decides it.
///
/// Collinear segments do **not** count as intersecting, which is the reason a
/// door placed along a corridor is invisible to RMF: the lane runs straight
/// through it and the reservation is never requested.
fn segments_intersect(a1: (f64, f64), a2: (f64, f64), b1: (f64, f64), b2: (f64, f64)) -> bool {
    fn orientation(p: (f64, f64), q: (f64, f64), r: (f64, f64)) -> i32 {
        let v = (q.1 - p.1) * (r.0 - q.0) - (q.0 - p.0) * (r.1 - q.1);
        if v.abs() < 1e-9 {
            0
        } else if v > 0.0 {
            1
        } else {
            2
        }
    }

    fn on_segment(p: (f64, f64), q: (f64, f64), r: (f64, f64)) -> bool {
        q.0 <= p.0.max(r.0) && q.0 >= p.0.min(r.0) && q.1 <= p.1.max(r.1) && q.1 >= p.1.min(r.1)
    }

    let o1 = orientation(a1, a2, b1);
    let o2 = orientation(a1, a2, b2);
    let o3 = orientation(b1, b2, a1);
    let o4 = orientation(b1, b2, a2);

    if o1 != o2 && o3 != o4 {
        return true;
    }
    // Touching endpoints count; a shared endpoint is how a lane meets the
    // door it passes through.
    (o1 == 0 && on_segment(a1, b1, a2))
        || (o2 == 0 && on_segment(a1, b2, a2))
        || (o3 == 0 && on_segment(b1, a1, b2))
        || (o4 == 0 && on_segment(b1, a2, b2))
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
            // Three elements: x, y, and an options map. The waypoint *name*
            // lives inside that map, and there is no theta slot at all —
            // `rmf_fleet_adapter/agv/parse_graph.cpp` reads `vertex[2]["name"]`
            // directly. A theta in the third slot, or a bare name in the
            // fourth, is read as a scalar where a mapping was expected, and
            // yaml-cpp reports the failure against whichever file it happened
            // to be opening, naming a key that is not in it.
            //
            // `is_charger` is in the same options map. RMF refuses to add a
            // robot to a fleet with no charging waypoint, so a vertex without
            // it (when it should have it) fails at `add_robot`, not here.
            let charger = if vertex.is_charger {
                ", is_charger: true"
            } else {
                ""
            };
            let _ = writeln!(
                out,
                "      - [{}, {}, {{name: {}{}}}]",
                yaml_number(vertex.x),
                yaml_number(vertex.y),
                yaml_string(&vertex.name),
                charger,
            );
        }
        out.push_str("    lanes:\n");
        for lane in &level.lanes {
            // The nav-graph parser reads a small, specific set of options and
            // ignores everything else. `graph_idx` and `bidirectional` are
            // building_map keys that this parser never looks up, so a lane
            // marked bidirectional there is still one-way here — which is why
            // both directions are emitted as separate lanes.
            //
            // `speed_limit` is read as a bare double, not a ParamValue pair.
            let mut params = format!("{{speed_limit: {}", yaml_number(lane.speed_limit));
            if let Some(orientation) = lane.orientation {
                // `orientation` is not a key this parser knows; it wants
                // `orientation_constraint` with a direction label.
                let label = if orientation.cos() >= 0.0 {
                    "forward"
                } else {
                    "backward"
                };
                let _ = write!(params, ", orientation_constraint: {}", yaml_string(label));
            }
            if let Some(door) = &lane.door_name {
                // Read as a plain string, not a `[type_code, value]` ParamValue
                // pair: `options["door_name"].as<std::string>()`.
                let _ = write!(params, ", door_name: {}", yaml_string(door));
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
    fn each_direction_of_an_edge_becomes_its_own_lane() {
        // The nav-graph parser has no bidirectional lane: `parse_graph.cpp`
        // calls `graph.add_lane(begin, end)` once per entry and reads no
        // `bidirectional` key. Collapsing a pair of opposite edges into one
        // lane would leave the reverse direction with no lane at all, so a
        // robot that drove somewhere could not come back.
        let map = small_map();
        let out = convert(&map, &Options::default()).expect("converts");
        assert_eq!(
            out.levels[0].lanes.len(),
            map.edges.len(),
            "every directed edge in the source is a lane"
        );

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
            doubled.edges.len(),
            "the reverse direction must gain a lane, not merge into one"
        );
    }

    #[test]
    fn a_map_without_chargers_has_no_charging_waypoints() {
        // RMF refuses to add a robot to a fleet whose nav graph has no
        // charging waypoint, so this has to stay absent unless asked for.
        let map = small_map();
        let out = convert(&map, &Options::default()).expect("converts");
        assert!(
            !out.levels[0].vertices.iter().any(|v| v.is_charger),
            "no charger listed, no is_charger emitted"
        );
    }

    #[test]
    fn listed_nodes_become_charging_waypoints() {
        let mut map = small_map();
        map.chargers = vec!["CONV-IN".to_owned(), "RK-AM".to_owned()];
        let out = convert(&map, &Options::default()).expect("converts");
        let level = &out.levels[0];

        let chargers: Vec<&str> = level
            .vertices
            .iter()
            .filter(|v| v.is_charger)
            .map(|v| v.name.as_str())
            .collect();
        assert_eq!(chargers, vec!["CONV-IN", "RK-AM"]);

        // A node that is not listed must not be marked, or every waypoint
        // would look like a charger and RMF would pick an arbitrary one.
        assert!(!level
            .vertices
            .iter()
            .any(|v| v.name == "RK-AW" && v.is_charger));
    }

    #[test]
    fn a_charger_that_is_not_a_node_is_ignored() {
        let mut map = small_map();
        map.chargers = vec!["NOPE".to_owned()];
        let out = convert(&map, &Options::default()).expect("converts");
        assert!(!out.levels[0].vertices.iter().any(|v| v.is_charger));
    }

    #[test]
    fn the_charger_flag_is_a_plain_bool_in_the_vertex_options() {
        // `parse_graph.cpp` reads `options["is_charger"].as<bool>()`, so it has
        // to be a bare YAML bool in the same options map as the name.
        let mut map = small_map();
        map.chargers = vec!["AS-1W".to_owned()];
        let out = convert(&map, &Options::default()).expect("converts");
        let yaml = to_yaml(&out);
        assert!(
            yaml.contains("- [0, 6, {name: AS-1W, is_charger: true}]"),
            "charger flag must sit next to the name in the options map, got:\n{yaml}"
        );
        assert_eq!(
            yaml.matches("is_charger").count(),
            1,
            "only the listed node"
        );
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
    fn a_vertex_carries_its_name_in_the_options_map() {
        // RMF's graph parser reads `vertex[2]["name"]`: the third element is
        // an options map, not a theta and not a bare name. Emitting theta in
        // that slot makes yaml-cpp look for `name` inside a scalar and fail,
        // blaming a key in the *fleet config* file — a completely different
        // file — which is why this is worth pinning by test.
        let map = small_map();
        let out = convert(&map, &Options::default()).expect("converts");
        let yaml = to_yaml(&out);
        let parsed: serde_yaml::Value = serde_yaml::from_str(&yaml).expect("emitted YAML parses");
        let level = &parsed["levels"]["L1"];
        let vertices = level["vertices"].as_sequence().expect("vertices sequence");
        for vertex in vertices {
            let fields = vertex.as_sequence().expect("vertex is a sequence");
            assert_eq!(fields.len(), 3, "vertex must be [x, y, options]: {yaml}");
            assert!(
                fields[2].is_mapping(),
                "third element must be an options map holding `name`: {yaml}"
            );
            assert!(
                fields[2]["name"].is_string(),
                "the name must be inside the options map: {yaml}"
            );
        }
    }

    #[test]
    fn the_door_name_is_a_plain_string_in_the_lane_options() {
        // `options["door_name"].as<std::string>()`: a `[type_code, value]`
        // ParamValue pair is read here as a sequence and the door is silently
        // dropped, so the reservation would never be requested.
        let map = small_map();
        let out = convert(&map, &Options::default()).expect("converts");
        let yaml = to_yaml(&out);
        let parsed: serde_yaml::Value = serde_yaml::from_str(&yaml).expect("emitted YAML parses");
        let level = &parsed["levels"]["L1"];
        let lanes = level["lanes"].as_sequence().expect("lanes sequence");
        let with_door: Vec<&serde_yaml::Value> = lanes
            .iter()
            .filter(|lane| lane[2].get("door_name").is_some())
            .collect();
        assert!(
            !with_door.is_empty(),
            "a lane through DOOR-IN must carry the door name: {yaml}"
        );
        for lane in with_door {
            assert!(
                lane[2]["door_name"].is_string(),
                "door_name must be a bare string, not a ParamValue pair: {yaml}"
            );
        }
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
