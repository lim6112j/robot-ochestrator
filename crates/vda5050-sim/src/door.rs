//! Shared-resource arbitration for the simulated fleet.
//!
//! A door is the canonical shared resource: two robots from different
//! manufacturers wanting the same doorway must not both be told to go. RMF
//! models this as a door with a state and a request protocol, and
//! `lif2rmf` already puts `door_name` on the lanes that cross each door. What
//! was missing is a robot that behaves accordingly.
//!
//! This module is that behaviour. A robot arriving at a door node asks for it;
//! if another robot holds it, the robot waits instead of driving through. The
//! door then stays open for a hold-off long enough for the holder to clear it,
//! which is what `deploy/rmf/door_adapter` does on the RMF side — the two
//! agree by construction because both follow the same rule.

use std::collections::HashMap;
use std::sync::RwLock;

/// A door robots contend for.
///
/// The node id is the map key, so it is not stored twice here.
#[derive(Debug, Clone)]
pub struct Door {
    /// Seconds the door stays open after being granted.
    pub hold_off_s: f64,
}

/// Which robot currently holds a door, and until when.
#[derive(Debug, Clone)]
struct Grant {
    /// `manufacturer:serialNumber` of the robot that was let through.
    holder: String,
    /// Simulated time at which the grant lapses.
    until_s: f64,
}

/// A [`DoorRegistry`] several robots can arbitrate through at once.
///
/// The registry is shared by the whole fleet — a door two robots want is one
/// physical doorway, so the decision cannot live inside a single robot — and
/// `RwLock` is what makes that safe without making every robot serialise on
/// every door. Reads (`is_door`, `holder`) take a read lock; only granting or
/// releasing takes the write lock.
#[derive(Debug, Default)]
pub struct SharedDoors(pub RwLock<DoorRegistry>);

impl SharedDoors {
    /// Build a shared registry.
    #[must_use]
    pub fn new(registry: DoorRegistry) -> Self {
        Self(RwLock::new(registry))
    }

    /// Whether a node id is a door.
    #[must_use]
    pub fn is_door(&self, node_id: &str) -> bool {
        self.0.read().expect("door registry lock").is_door(node_id)
    }

    /// Who holds a door right now, if anyone.
    #[must_use]
    pub fn holder(&self, node_id: &str) -> Option<String> {
        self.0
            .read()
            .expect("door registry lock")
            .holder(node_id)
            .map(str::to_owned)
    }

    /// Ask to pass through a door.
    #[must_use]
    pub fn request(&self, node_id: &str, robot_id: &str, now_s: f64) -> bool {
        self.0
            .write()
            .expect("door registry lock")
            .request(node_id, robot_id, now_s)
    }

    /// Give up a door before leaving it.
    pub fn release(&self, node_id: &str, robot_id: &str) {
        self.0
            .write()
            .expect("door registry lock")
            .release(node_id, robot_id);
    }
}

/// Arbitrates access to every door in the building.
///
/// One instance is shared by the whole fleet, which is the point: a door two
/// robots want is one physical doorway, so the decision cannot live inside a
/// single robot.
#[derive(Debug, Default)]
pub struct DoorRegistry {
    doors: HashMap<String, Door>,
    grants: HashMap<String, Grant>,
}

impl DoorRegistry {
    /// Register a door.
    pub fn add(&mut self, node_id: impl Into<String>, hold_off_s: f64) {
        self.doors.insert(node_id.into(), Door { hold_off_s });
    }

    /// Whether a node id is a door.
    #[must_use]
    pub fn is_door(&self, node_id: &str) -> bool {
        self.doors.contains_key(node_id)
    }

    /// How many doors are registered.
    #[must_use]
    pub fn len(&self) -> usize {
        self.doors.len()
    }

    /// Who holds a door right now, if anyone.
    #[must_use]
    pub fn holder(&self, node_id: &str) -> Option<&str> {
        self.grants.get(node_id).map(|g| g.holder.as_str())
    }

    /// Give up a door before leaving it.
    ///
    /// Only the holder may release, and releasing a door the caller does not
    /// hold is a no-op rather than a free door for everyone.
    pub fn release(&mut self, node_id: &str, robot_id: &str) {
        if let Some(grant) = self.grants.get(node_id) {
            if grant.holder == robot_id {
                self.grants.remove(node_id);
            }
        }
    }

    /// Ask to pass through a door.    ///
    /// Returns `true` when the robot may proceed. A robot that already holds
    /// the door is always let through — it is the same doorway it just entered.
    ///
    /// `now_s` is simulated time, so the registry stays deterministic.
    #[must_use]
    pub fn request(&mut self, node_id: &str, robot_id: &str, now_s: f64) -> bool {
        let Some(door) = self.doors.get(node_id) else {
            // Not a door: nothing to arbitrate, so it is never a reason to wait.
            return true;
        };
        if let Some(grant) = self.grants.get(node_id) {
            if grant.holder == robot_id {
                return true;
            }
            if grant.until_s > now_s {
                return false;
            }
        }
        let hold_off_s = door.hold_off_s;
        self.grants.insert(
            node_id.to_owned(),
            Grant {
                holder: robot_id.to_owned(),
                until_s: now_s + hold_off_s,
            },
        );
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn registry() -> DoorRegistry {
        let mut registry = DoorRegistry::default();
        registry.add("DOOR-IN", 6.0);
        registry.add("DOOR-OUT", 6.0);
        registry
    }

    #[test]
    fn a_door_is_identified_by_its_node_id() {
        let mut registry = registry();
        assert!(registry.is_door("DOOR-IN"));
        assert!(registry.is_door("DOOR-OUT"));
        assert!(!registry.is_door("AS-1M"));
        // A node that is not a door is never a reason to wait.
        assert!(registry.request("AS-1M", "acme:AMR-001", 0.0));
    }

    #[test]
    fn the_first_robot_through_gets_the_door() {
        let mut registry = registry();
        assert!(registry.request("DOOR-IN", "acme:AMR-001", 0.0));
        assert_eq!(registry.holder("DOOR-IN"), Some("acme:AMR-001"));
    }

    #[test]
    fn a_second_robot_is_held_back() {
        // Two different manufacturers, one doorway: the whole point.
        let mut registry = registry();
        assert!(registry.request("DOOR-IN", "acme:AMR-001", 0.0));
        assert!(
            !registry.request("DOOR-IN", "sweepco:SCR-001", 1.0),
            "a second robot must wait while the first holds the door"
        );
    }

    #[test]
    fn the_holder_may_always_pass() {
        // Re-entering the doorway it just came through is not a conflict.
        let mut registry = registry();
        assert!(registry.request("DOOR-IN", "acme:AMR-001", 0.0));
        assert!(registry.request("DOOR-IN", "acme:AMR-001", 3.0));
    }

    #[test]
    fn the_door_frees_after_the_hold_off() {
        let mut registry = registry();
        assert!(registry.request("DOOR-IN", "acme:AMR-001", 0.0));
        assert!(!registry.request("DOOR-IN", "sweepco:SCR-001", 5.9));
        // Hold-off is 6s; at exactly 6s the grant has lapsed and a later
        // robot may take the door. No explicit expiry call is needed because
        // `request` treats a lapsed grant as absent.
        assert!(registry.request("DOOR-IN", "sweepco:SCR-001", 6.0));
        assert_eq!(registry.holder("DOOR-IN"), Some("sweepco:SCR-001"));
    }

    #[test]
    fn two_doors_are_independent() {
        // A robot held at DOOR-IN must not block a robot at DOOR-OUT.
        let mut registry = registry();
        assert!(registry.request("DOOR-IN", "acme:AMR-001", 0.0));
        assert!(registry.request("DOOR-OUT", "sweepco:SCR-001", 0.0));
        assert!(!registry.request("DOOR-IN", "sweepco:SCR-001", 0.0));
    }
}
