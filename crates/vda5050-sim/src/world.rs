//! Map graph used by the simulator to travel between nodes.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

use serde::Deserialize;
use vda5050_types::types::Position;

/// A node of the map graph.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MapNode {
    pub x: f64,
    pub y: f64,
    #[serde(default)]
    pub theta: f64,
}

/// A drivable edge of the map graph.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MapEdge {
    pub from: String,
    pub to: String,
    /// Edge length in metres.
    #[serde(default)]
    pub length: Option<f64>,
    /// Maximum speed in m/s used to compute travel time.
    #[serde(default = "default_speed")]
    pub speed: f64,
}

fn default_speed() -> f64 {
    1.0
}

/// A whole map.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MapGraph {
    pub map_id: String,
    pub nodes: HashMap<String, MapNode>,
    #[serde(default)]
    pub edges: Vec<MapEdge>,
}

impl MapGraph {
    /// Map identifier.
    #[must_use]
    pub fn map_id(&self) -> &str {
        &self.map_id
    }

    /// Load a map from JSON.
    pub fn load(path: &std::path::Path) -> Result<Self, WorldError> {
        let raw = std::fs::read_to_string(path).map_err(|source| WorldError::Io {
            path: path.display().to_string(),
            source,
        })?;
        serde_json::from_str(&raw).map_err(WorldError::Json)
    }

    /// Position of a node.
    #[must_use]
    pub fn position(&self, node_id: &str) -> Option<Position> {
        self.nodes.get(node_id).map(|n| Position {
            x: n.x,
            y: n.y,
            theta: n.theta,
            map_id: self.map_id.clone(),
            localized: true,
            localization_score: Some(1.0),
            deviation_range: None,
        })
    }

    /// Cost of one edge in seconds; `None` when the edge does not exist.
    fn edge_cost(&self, from: &str, to: &str) -> Option<f64> {
        self.edges
            .iter()
            .find(|e| e.from == from && e.to == to)
            .map(|e| {
                let length = e.length.unwrap_or_else(|| self.distance(from, to));
                length / e.speed.max(0.1)
            })
    }

    /// Travel time along the fastest drivable route (Dijkstra).
    ///
    /// Returns 0.0 only for identical nodes or when no route exists — the
    /// caller treats that as "already there".
    #[must_use]
    pub fn travel_time(&self, from: &str, to: &str) -> f64 {
        self.route(from, to).map_or(0.0, |r| r.cost_s)
    }

    /// Fastest route between two nodes as an ordered list of node ids.
    #[must_use]
    #[cfg(test)]
    pub fn node_route(&self, from: &str, to: &str) -> Vec<String> {
        self.route(from, to).map_or_else(Vec::new, |r| r.nodes)
    }

    /// Dijkstra over the drivable edge graph.
    fn route(&self, from: &str, to: &str) -> Option<Route> {
        if from == to {
            return Some(Route {
                cost_s: 0.0,
                nodes: vec![from.to_owned()],
            });
        }
        // `BinaryHeap` is a max-heap, so the cost is wrapped in `Reverse` to pop
        // the cheapest node first.
        let mut dist: HashMap<&str, f64> = HashMap::new();
        let mut prev: HashMap<&str, &str> = HashMap::new();
        let mut heap: BinaryHeap<(Reverse<ordered_float::OrderedFloat<f64>>, &str)> =
            BinaryHeap::new();
        dist.insert(from, 0.0);
        heap.push((Reverse(ordered_float::OrderedFloat(0.0)), from));

        while let Some((Reverse(ordered_float::OrderedFloat(cost)), node)) = heap.pop() {
            if node == to {
                let mut nodes = vec![node.to_owned()];
                let mut cur = node;
                while let Some(&p) = prev.get(cur) {
                    nodes.push(p.to_owned());
                    cur = p;
                }
                nodes.reverse();
                return Some(Route {
                    cost_s: cost,
                    nodes,
                });
            }
            if cost > *dist.get(node).unwrap_or(&f64::INFINITY) {
                continue;
            }
            for edge in self.edges.iter().filter(|e| e.from == node) {
                if !self.nodes.contains_key(edge.to.as_str()) {
                    continue;
                }
                let next_cost = cost + self.edge_cost(node, &edge.to).unwrap_or(f64::INFINITY);
                if next_cost < *dist.get(edge.to.as_str()).unwrap_or(&f64::INFINITY) {
                    dist.insert(edge.to.as_str(), next_cost);
                    prev.insert(edge.to.as_str(), node);
                    heap.push((
                        Reverse(ordered_float::OrderedFloat(next_cost)),
                        edge.to.as_str(),
                    ));
                }
            }
        }
        None
    }

    /// Polyline (in map coordinates) of the fastest route, including `from` and
    /// `to` as first/last point. Empty when no route exists.
    #[must_use]
    pub fn polyline(&self, from: &str, to: &str) -> Vec<(f64, f64)> {
        self.route(from, to).map_or_else(Vec::new, |r| {
            r.nodes
                .iter()
                .filter_map(|n| self.nodes.get(n))
                .map(|n| (n.x, n.y))
                .collect()
        })
    }

    /// Straight-line distance between two nodes.
    #[must_use]
    pub fn distance(&self, from: &str, to: &str) -> f64 {
        match (self.nodes.get(from), self.nodes.get(to)) {
            (Some(a), Some(b)) => (b.x - a.x).hypot(b.y - a.y),
            _ => 0.0,
        }
    }
}

/// A computed route.
#[derive(Debug, Clone)]
struct Route {
    cost_s: f64,
    nodes: Vec<String>,
}

/// World loading errors.
#[derive(Debug, thiserror::Error)]
pub enum WorldError {
    #[error("cannot read map {path}: {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("invalid map json: {0}")]
    Json(#[from] serde_json::Error),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn graph() -> MapGraph {
        serde_json::from_str(
            r#"{"mapId":"plant-a/l1",
                "nodes":{"A":{"x":0,"y":0},"B":{"x":3,"y":4}},
                "edges":[{"from":"A","to":"B","length":5.0,"speed":1.25}]}"#,
        )
        .expect("valid map")
    }

    #[test]
    fn loads_nodes_and_edges() {
        let g = graph();
        assert_eq!(g.map_id, "plant-a/l1");
        assert_eq!(g.travel_time("A", "B"), 4.0);
        assert_eq!(g.distance("A", "B"), 5.0);
        assert_eq!(g.travel_time("B", "A"), 0.0);
    }

    #[test]
    fn routes_through_intermediate_nodes() {
        // A -> B 가 직통이 없어도 A -> C -> B 로 우회한다.
        let g: MapGraph = serde_json::from_str(
            r#"{"mapId":"m","nodes":{"A":{"x":0,"y":0},"C":{"x":1,"y":0},"B":{"x":2,"y":0}},
                "edges":[{"from":"A","to":"C","length":1,"speed":1},
                         {"from":"C","to":"B","length":1,"speed":1}]}"#,
        )
        .expect("valid map");
        assert_eq!(g.travel_time("A", "B"), 2.0);
        assert_eq!(g.node_route("A", "B"), vec!["A", "C", "B"]);
        assert_eq!(
            g.travel_time("B", "A"),
            0.0,
            "역방향 엣지가 없으면 경로 없음"
        );
    }

    #[test]
    fn picks_the_faster_of_two_routes() {
        let g: MapGraph = serde_json::from_str(
            r#"{"mapId":"m","nodes":{"A":{"x":0,"y":0},"S":{"x":1,"y":0},"L":{"x":1,"y":0},"B":{"x":2,"y":0}},
                "edges":[{"from":"A","to":"S","length":1,"speed":10},
                         {"from":"S","to":"B","length":1,"speed":10},
                         {"from":"A","to":"L","length":1,"speed":0.1},
                         {"from":"L","to":"B","length":1,"speed":0.1}]}"#,
        )
        .expect("valid map");
        assert_eq!(g.travel_time("A", "B"), 0.2);
        assert_eq!(g.node_route("A", "B"), vec!["A", "S", "B"]);
    }

    #[test]
    fn position_carries_map_id() {
        let g = graph();
        let p = g.position("B").expect("node exists");
        assert_eq!(p.x, 3.0);
        assert_eq!(p.map_id, "plant-a/l1");
        assert!(p.localized);
    }
}
