# VDA 5050 door adapter for RMF

Publishes RMF `door_states` and answers `door_requests` for the doors that
`lif2rmf` models out of the VDA 5050 map. This is what turns a node named
`DOOR-IN` into a resource robots must reserve.

## Why a separate adapter

A door is a shared resource: two robots from different fleets wanting the same
doorway must not both be told to go. RMF models that as a door with a state
(`MODE_OPEN` / `MODE_CLOSED` / …) and a request protocol, and it holds a robot
back until the door is open and the previous robot has finished passing.

The doors already exist in the nav graph — `lif2rmf` emits them, and the lanes
that cross them carry `door_name`. What was missing was the adapter that
publishes their state and grants requests.

## The one non-obvious part

`MODE_OPEN` does not mean "physically open". It means "available", and the
adapter is what decides. Publishing `MODE_CLOSED` forever would deadlock every
mission that crosses a door; publishing `MODE_OPEN` unconditionally would
defeat the reservation entirely. So the adapter opens a door when asked and
keeps it open for a hold-off period long enough for the requesting robot to
clear it, which is the behaviour RMF's own examples use.

## Running it

```bash
docker build -f deploy/rmf/Dockerfile -t rcs-rmf:jazzy deploy/rmf
docker run --rm -it --network host \
  -v "$PWD/deploy/rmf/door_adapter:/rmf/door_adapter" \
  rcs-rmf:jazzy
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run vda5050_door_adapter door_adapter -c /rmf/door_adapter/config.yaml
```

## Verified

See the E2E notes in `PLAN.md`: two robots from different manufacturers
requesting the same door, one admitted and the other held, observed through
the control dashboard.

## What is verified, end to end

Both halves now exist and were run against the live simulator and broker.

**RMF side** (container):

* `door_states` is published, `DOOR-IN` and `DOOR-OUT` both start `MODE_CLOSED`.
* A `door_requests` message opens the door (`MODE_OPEN`) and logs who asked.
* After the hold-off the door closes again, so the next robot must ask.
* A request for an unknown door is refused with a warning and the state is
  unchanged.

**Robot side** (`crates/vda5050-sim/src/door.rs`, `robot.rs`):

Two robots from different manufacturers (`acme:AMR-002` and
`sweepco:SCR-001`) were both sent a mission whose first node is `DOOR-IN`. The
simulator log shows the second robot being held and then released:

```
waiting for door held by another robot robot=sweepco:SCR-001 door=DOOR-IN holder=acme:AMR-002
door granted, resuming robot=sweepco:SCR-001 door=DOOR-IN
```

and the control screen showed the handover through `/api/shared-resources`:

```
t+ 1s  ['DOOR-IN<-AMR-002']
t+15s  ['DOOR-IN<-AMR-002', 'DOOR-IN<-SCR-001']   ← second robot waiting
t+21s  ['DOOR-IN<-SCR-001']                        ← first has left
```

## How the wait reaches the dashboard

VDA 5050 v3 has no field for "waiting for a shared resource", and a robot
stopped at a doorway is indistinguishable from one that lost its connection in
a position plot. The simulator therefore reports the wait as a
`WAITING_FOR_DOOR` entry in `state.errors`, which `errorType` permits because
it is an extensible string. `RobotAdapter::holds()` reads it, so
`/api/shared-resources` can say who is held and why.

Two details there are easy to get wrong and are pinned by tests:

* `errorReferences` is an array of `{referenceKey, referenceValue}` objects,
  **not** bare strings. A string passes Rust's type check but fails schema
  validation, which drops the whole `state` message — the gateway's
  `messages_rejected` going non-zero is the symptom.
* The hold-off has to be long enough to observe. The default 6s made the wait
  a 0.25s window that polling missed; `deploy/sim-warehouse.yaml` sets 25s so
  the handover is visible.
