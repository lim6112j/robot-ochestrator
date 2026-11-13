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

## What is verified, and what is not

Verified in the container:

* `door_states` is published, `DOOR-IN` and `DOOR-OUT` both start `MODE_CLOSED`.
* A `door_requests` message opens the door (`MODE_OPEN`) and logs who asked.
* After the hold-off the door closes again, so the next robot must ask.
* A request for an unknown door is refused with a warning and the state is
  unchanged.

Not yet verified: a robot actually *waiting*. The simulator
(`crates/vda5050-sim`) has no notion of a door — it treats `DOOR-*` as an
ordinary node and drives straight through. So the reservation protocol is
correct on the RMF side, but nothing downstream of it observes the door yet,
and no robot is ever held back.

That gap is the remaining work for Phase 2: the simulator needs to honour
`door_name` on its lanes, either by refusing to enter a closed door or by
asking and waiting. Until then the door adapter is exercised but not
load-bearing, and this README says so rather than implying otherwise.
