# RMF container (Phase 2)

ROS 2 **Jazzy** + Open-RMF on Ubuntu 24.04. Built from the standard ROS 2 apt
repository — no extra source list or keyring is needed, because RMF publishes
binaries there.

## Why this image exists

ADR-004 (see `PLAN.md` §6.2) settled four things:

| Decision | Value | Why |
|---|---|---|
| ROS 2 distro | Jazzy | RMF ships binaries for Jazzy only. Lyrical lasts longer (2031 vs 2029) but has no RMF packages, which would mean building RMF from source. |
| Boundary language | Python `fleet_adapter_template` | The adapter is a thin, replaceable translation layer; the reference implementation already carries the traffic-negotiation logic. Rust keeps the gateway, API and UI. |
| Control level | Full Control | RMF plans paths and speeds; exposed as a runtime setting. |
| Environment | Docker | The dev host is macOS and ROS 2/RMF need Linux. |

The Rust-only rule is knowingly broken for this one component, and the
boundary is a trait so the adapter can be swapped for a Rust one later.

## Build and run

```bash
docker build -f deploy/rmf/Dockerfile -t rcs-rmf:jazzy deploy/rmf
docker run --rm -it --network host rcs-rmf:jazzy
```

`--network host` matters: the fleet adapter will talk to the MQTT broker and
to the VDA 5050 simulator on the host.

## Traffic arbitration: negotiation starts

`traffic_probe.py` submits two conflicting itineraries on one lane, from two
participants standing in for robots of different manufacturers, and reports
what the schedule does with them.

Verified: the negotiation starts. The schedule node logs
`Require negotiation for: 0 1`, and the negotiation topics carry
`notice: 1`, `participants: [0, 1]`, and a state tree with a node for each
participant.

Not verified: the negotiation's *conclusion*. That needs something to answer
the schedule's proposals, and the probe only impersonates a participant. The
answerer is the fleet adapter.

Two things that cost most of the debugging time, both silent:

* Waypoint `time` in an `ItinerarySet` is an **absolute nanosecond** timestamp
  on the steady_clock. Sending `0, 10, 20, 30, 40` reads as one instant in the
  distant past and the whole itinerary is dropped with no warning. Anchor on
  `now + t * 1e9`.
* The map name is the **level** name. Our building map declares
  `levels: L1:`, so routes must say `L1`; `plant-b/l1` is a name the schedule
  has never seen and the route is ignored, again silently.

An earlier note here claimed the schedule's failure to subscribe to `/map`
was the blocker. That is true — it reports `Subscription count: 0` — but it is
by design: the schedule arbitrates in `(map, x, y, t)` space and does not need
the nav graph. `rmf_traffic_editor` is not needed either; the official
`rmf_demos` launch runs `building_map_server` + `rmf_traffic_schedule` +
`rmf_traffic_blockade` with no editor.

## Running the fleet adapter

```
pip3 install --break-system-packages nudged
export PYTHONPATH=/path/to/deploy/rmf/fleet_adapter:$PYTHONPATH
python3 -m vda5050_fleet_adapter.fleet_adapter \
  -c .../vda5050_fleet_adapter/config.yaml \
  -n .../nav_graph.yaml
```

Two things to know:

* `nudged` is not in the image and `pip3` refuses it under PEP 668, so
  `--break-system-packages` is required.
* `PYTHONPATH` must *append* to the ROS path. Setting it alone shadows
  `rclpy` and the adapter dies on `import rclpy`.

The nav graph and the building map are the same file: `nav_graph.yaml` is a
copy of the `lif2rmf` output, because the two parsers now agree on one format.

Once it starts, the schedule node logs
`Mirror handling new sync of 1 queries from schedule node`, which is the point
at which the nav graph and the fleet adapter are actually talking to each
other.

Past that point the adapter currently dies — see "Fleet adapter: stops after
the nav graph is accepted" below.

## Fleet adapter: runs

`deploy/rmf/fleet_adapter` starts and stays up against the real schedule node:

```
Finished configuring Easy Full Control adapter for fleet [vda5050Fleet]
Mirror handling new sync of 1 queries from schedule node
Requesting new schedule update because update timed out
```

### The bug was a double unwrap

`RobotAPI.__init__` reads `config_yaml['fleet_manager']` itself, but
`fleet_adapter.py` was passing `config_yaml['fleet_manager']` to it — the
already-unwrapped section. So the constructor went looking for
`fleet_manager.fleet_manager`, which is not there.

It presented as a **segfault**, not a `KeyError`, which is why it took so long:
the RMF adapter's executor thread is in FastDDS at the time, and by the time
Python unwinds, the stack is gone. `gdb` names it plainly —
`pthread_mutex_lock` under `DataReaderHistory::get_first_untaken_info` under
`rclcpp::Executor::wait_for_work` — and that C++ stack is a red herring. It is
where the *other* thread happens to be, not what broke.

Two traps worth recording, both of which produced wrong diagnoses:

* `sys.path.insert(0, ...)` plus `import RobotClientAPI` segfaults on its own,
  with no RMF adapter running. Import it as a package or load the file with
  `importlib` and it works. While unaware of this, four plausible causes were
  identified — paho's callback API version, `mqtt.Client` after
  `Adapter.start()`, the `nudged` identity transform, and
  `get_known_robot_configuration` on a name with a colon — and every one passes
  when reproduced properly. None is the cause.
* A misleading `KeyError` is reported from a line that ran correctly, because
  Python unwinds a dead stack. Printing the keys immediately before shows they
  are all there.

`gdb` is now in the image (`apt-get install -y gdb`; it is in neither the base
image nor on PyPI), which is what finally named the real stack.

### Running it

```
pip3 install --break-system-packages nudged
export PYTHONPATH=/path/to/deploy/rmf/fleet_adapter:$PYTHONPATH
python3 -m vda5050_fleet_adapter.fleet_adapter \
  -c .../vda5050_fleet_adapter/config.yaml \
  -n .../nav_graph.yaml
```

`PYTHONPATH` must *append* to the ROS path; setting it alone shadows `rclpy`.

The image also ships a pip-installed copy of the adapter under
`/usr/local/lib/python3.12/dist-packages`, which shadows a mounted source tree
and is an older revision. Delete it when iterating on the source:
`rm -rf /usr/local/lib/python3.12/dist-packages/vda5050_fleet_adapter`.

### Not yet verified

Participants are **not** registered — the log shows 0. The adapter subscribes
to robot `state` over MQTT and only registers a participant once it has seen
one, so this needs the simulator publishing to the broker at the same time.
Traffic-negotiation conclusions therefore remain unverified.

## Verified

* 42 `ros-jazzy-rmf-*` packages installed.
* `rmf_traffic_ros2 rmf_traffic_schedule` runs and advertises the traffic
  topics: `/rmf_traffic/participants`, `/rmf_traffic/negotiation_*` (notice,
  proposal, rejection, refusal, forfeit, ack, conclusion, states, statuses),
  `/rmf_traffic/itinerary_*` (set, extend, delay, reached, clear) and
  `/rmf_traffic/heartbeat`.
* `rmf_building_map_tools` is present for authoring the nav graph.
* `rmf_traffic_editor` is **not** usable here: only `rmf_traffic_editor_assets`
  is installed, with no editor binary, and it could not run headless anyway.
  It is not needed — see the traffic arbitration section.

Note the executable name is `rmf_traffic_schedule`, not `schedule`.

Also confirmed, and worth recording because it contradicts a reasonable
assumption: the schedule node does **not** subscribe to `/map`
(`ros2 topic info /map` reports `Subscription count: 0`, and the QoS profile
is a plain RELIABLE / TRANSIENT_LOCAL). This is by design. It arbitrates in
`(map, x, y, t)` space and never needs the nav graph.

## Network prerequisite

The OrbStack VM needs internet egress. If `docker pull` hangs, check
`orbctl config get network_proxy`: a stale value pointing at a dead host
(e.g. a removed minikube gateway) silently kills all VM egress while DNS and
connections to OrbStack's own proxy keep working. `orbctl config set
network_proxy none` plus a restart fixes it when the host itself has direct
internet.
