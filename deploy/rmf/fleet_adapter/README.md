# VDA 5050 fleet adapter for RMF

A `full_control` RMF fleet adapter whose `RobotClientAPI` talks **VDA 5050 over
MQTT** instead of a vendor REST API. It is the RMF boundary from ADR-004.

## Why this is Python

ADR-004 (`PLAN.md` §6.2) decided the boundary language is Python
`fleet_adapter_template`, which knowingly breaks the Rust-only rule for this
one component. The adapter is a thin, replaceable translation layer between
RMF messages and VDA 5050 MQTT; the reference implementation already carries
the traffic-negotiation logic, and `rclrs` still declares "no stability
guarantees". Rust keeps the gateway, API, UI and mission issuing.

The boundary is a trait on the Rust side, so this adapter can be swapped for a
Rust one later without touching anything else.

## What it does

| RMF side | VDA 5050 side |
|---|---|
| `navigate(pose)` | publish an `order` with one node at that pose |
| `stop(robot)` | publish `instantActions` with `cancelOrder` |
| `position()` / `battery_soc()` | read from the robot's `state` |
| `localize()` | publish `initPosition` |
| activity (pick/drop/…) | a node action in the `order` |

Robot state is read from MQTT, so the adapter never polls a robot directly:
the same `state` the gateway ingests is the source of truth, which keeps the
dashboard and RMF from disagreeing about where a robot is.

## Running it

`nav_graph.yaml` is generated, not checked in — regenerate it from the map:

```bash
cargo run -p lif2rmf -- --map deploy/maps/plant-b.l1.json \
  --out deploy/rmf/fleet_adapter/nav_graph.yaml
```

The adapter lives in the RMF container (`deploy/rmf/Dockerfile` installs
`rmf_fleet_adapter_python`); this directory is mounted in.

```bash
docker build -f deploy/rmf/Dockerfile -t rcs-rmf:jazzy deploy/rmf
docker run --rm -it --network host \
  -v "$PWD/deploy/rmf/fleet_adapter:/rmf/fleet_adapter" \
  rcs-rmf:jazzy
```

Then, inside the container:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run vda5050_fleet_adapter fleet_adapter \
  -c /rmf/fleet_adapter/config.yaml \
  -n /rmf/fleet_adapter/nav_graph.yaml \
  -s ws://0.0.0.0:7878
```

## Verified

Driven against the live simulator and broker, then the captured messages run
through the repo's own conformance tool:

* `check_connection()` → True after `connect()`.
* `position()` / `battery_soc()` / `get_map_name()` / `get_data()` return the
  same values the gateway shows for the same robot — both read the same
  `state` messages, so they cannot disagree.
* `navigate()` publishes an `order` with one node at the requested pose, on
  `vda5050/v3/<manufacturer>/<serialNumber>/order`.
* `stop()` publishes `cancelOrder`, `localize()` publishes `initPosition`.
* Both message kinds pass `conformance check-message` against the official
  v3.0.0 schemas.

Three bugs found while verifying, all of which fail quietly:

1. `connect()` passed `self.prefix` (`host:port`) as the host, so
   `getaddrinfo` failed on the colon. The host has to be split off.
2. The subscription built `vda5050/v3/acme:AMR-001/state` from a
   `manufacturer:serialNumber` config value — a colon where the topic needs a
   slash. The subscription matched nothing, so every getter returned `None`
   and the adapter looked merely "not connected yet" rather than broken.
3. `instantActions` messages were sent without the header, and with the action
   at the top level instead of inside `actions[]`. Both are schema violations
   the schema check catches immediately, which is why the check runs here.

The header is now built by one helper (`_header`) instead of being repeated in
four places, which is what let it drift in the first place.
