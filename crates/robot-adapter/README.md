# robot-adapter — the fleet adapter contract

`RobotAdapter` is the boundary between the gateway and any fleet adapter. It
exists so ADR-004's Python choice stays a *replaceable part* rather than
load-bearing architecture.

## The trait

Four things an adapter must provide:

| Method | Purpose |
|---|---|
| `manufacturer()` | Which fleet this adapter speaks for |
| `robots()` | Robots it currently knows about, as canonical models |
| `dispatch(robot, order)` | Send a mission |
| `instant_action(robot, actions)` | Pause, resume, cancel |
| `shared_resources()` | Doors and lifts robots contend for |
| `holds(robot, resource)` | Whether a robot currently occupies one |
| `observe_state()` / `observe_connection()` | Optional; the gateway feeds it what it ingests |

`AdapterError` separates `Dispatch` (the robot refused — nothing is running)
from `Transport` (could not reach — the operator cannot tell). Collapsing
those two loses information an operator needs.

## Implementations

* `mqtt::MqttRobotAdapter` — VDA 5050 over MQTT, the Rust counterpart of
  `deploy/rmf/fleet_adapter`. Observe-only for dispatch: it reports state but
  refuses to dispatch, because publishing is the ingest loop's job and an
  adapter that accepted a mission it could not send would be lying.
* `NullAdapter` — answers for nobody. A fleet with nothing behind it yet
  reports no robots instead of pretending.

## Registering

```bash
gateway --adapter-manufacturer acme --adapter-manufacturer sweepco
```

Repeatable, optional, and env-backed (`ADAPTER_MANUFACTURER`). The gateway
still dispatches over MQTT itself; adapters answer "what can this fleet do".

`GET /api/adapters` lists them, `GET /api/shared-resources` lists doors with
which robot holds each. The latter is how a dashboard explains a stopped
robot: a robot waiting at a door and a robot that lost its connection look
identical in a position plot.

## Two things that failed quietly

1. `observe_state` was an inherent method, not part of the trait impl, so
   through `Arc<dyn RobotAdapter>` the trait's no-op default ran and the
   adapter never saw a robot. It looked exactly like "no robots yet".
2. `robots()` keys observations by serial number, but a `state` message
   carries its own manufacturer. Without checking it, an adapter reports
   another fleet's robots and the coordination layer negotiates for a fleet
   it does not speak for.
