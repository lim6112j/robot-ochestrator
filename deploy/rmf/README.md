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

## Traffic arbitration: not yet verified

`traffic_probe.py` is an attempt to verify the other half of the Phase 2
completion criterion — two robots from different manufacturers negotiating a
shared aisle. It does not reach a negotiation, and the reason is pinned down:

* `rmf_traffic_schedule` starts, and accepts participant registration,
  itinerary submission, live participant state and progress reporting.
* It does **not** subscribe to `/map`. `ros2 topic info /map` reports
  `Subscription count: 0` even after 25s of discovery.
* Without a nav graph there is nothing to arbitrate, so no
  `/rmf_traffic/negotiation_*` traffic ever appears.

The missing piece is whatever turns a `.building.yaml` into a schedule
nav-graph patch. In RMF that is `rmf_traffic_editor`, but this image carries
only `rmf_traffic_editor_assets` — the Qt editor binary is absent and cannot
run headless. Closing this needs either a source build of
`rmf_traffic_editor` or composing `SchedulePatch` messages directly.

The probe's docstring records this so the same ground is not re-dug.

## Verified

* 42 `ros-jazzy-rmf-*` packages installed.
* `rmf_traffic_ros2 rmf_traffic_schedule` runs and advertises the traffic
  topics: `/rmf_traffic/participants`, `/rmf_traffic/negotiation_*` (notice,
  proposal, rejection, refusal, forfeit, ack, conclusion, states, statuses),
  `/rmf_traffic/itinerary_*` (set, extend, delay, reached, clear) and
  `/rmf_traffic/heartbeat`.
* `rmf_traffic_editor` and `rmf_building_map_tools` are present for authoring
  the nav graph.

Note the executable name is `rmf_traffic_schedule`, not `schedule`.

## Network prerequisite

The OrbStack VM needs internet egress. If `docker pull` hangs, check
`orbctl config get network_proxy`: a stale value pointing at a dead host
(e.g. a removed minikube gateway) silently kills all VM egress while DNS and
connections to OrbStack's own proxy keep working. `orbctl config set
network_proxy none` plus a restart fixes it when the host itself has direct
internet.
