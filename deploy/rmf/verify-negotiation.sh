#!/usr/bin/env bash
# Report on RMF traffic arbitration with this project's own fleet adapter as
# the participant that answers.
#
# What it asserts, and what it only reports
# -----------------------------------------
# ASSERTS (a regression here is a real failure):
#   * the artifacts build, the broker answers, the simulator and adapter run
#   * the adapter registers both participants
#   * the schedule node is reachable and mirroring
#
# REPORTS WITHOUT ASSERTING:
#   * whether a negotiation starts
#   * whether it concludes with resolved: true
#
# The reason for the split is that a conclusion is not currently reachable on
# demand. This adapter is built on the upstream template, which expects
# `rmf_task_dispatcher` to turn a delivery request into an RMF task; that
# component is not part of this project and is not started here. What does
# produce itineraries is the adapter's own parking task, so a negotiation
# happens only when a robot's live driving happens to overlap it. That is a
# timing coincidence, not a path this project controls, so asserting on it
# would make the script fail at random.
#
# The negotiation *mechanism* is verified separately and deterministically by
# `deploy/rmf/traffic_probe.py`, which submits conflicting itineraries itself.
# What is missing is the last hop — the adapter answering a proposal for a
# task it was given — and closing that needs the dispatch path, not a script.
#
# Exits 0 when the assertable part passes. Use --strict to also require a
# concluded negotiation, which is what CI should eventually do once the
# dispatch path exists.

set -uo pipefail

# ── configuration ────────────────────────────────────────────────────────────
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT" || exit 1

IMAGE="${RCS_RMF_IMAGE:-rcs-rmf:jazzy}"
DOMAIN="${ROS_DOMAIN_ID:-42}"
BROKER="${RCS_BROKER:-localhost:1883}"
BROKER_HOST="${BROKER%%:*}"
BROKER_PORT="${BROKER##*:}"
MAP="deploy/maps/plant-b.l1.lif.json"
SIM_CONFIG="deploy/sim-warehouse.yaml"
GATEWAY_TOKEN="${RCS_API_TOKEN:-test-secret-123}"

# The adapter takes ~20s to dispatch its own parking tasks, and the negotiation
# needs another ~25s to conclude. Generous, because the point is to catch
# regressions rather than to be fast.
STRICT=0
for arg in "$@"; do
    case "$arg" in
        --strict) STRICT=1 ;;
        *) die "unknown argument: $arg (only --strict is accepted)" ;;
    esac
done

ADAPTER_WARMUP_S="${ADAPTER_WARMUP_S:-30}"
CONCLUSION_TIMEOUT_S="${CONCLUSION_TIMEOUT_S:-45}"

WORK="$(mktemp -d)"
# Overridable so a failing run's logs survive cleanup.
LOG_DIR="${LOG_DIR:-$WORK}"
mkdir -p "$LOG_DIR" || { echo "cannot create $LOG_DIR" >&2; exit 1; }

PIDS=()
cleanup() {
    for pid in ${PIDS+"${PIDS[@]}"}; do kill "$pid" 2>/dev/null; done
    docker rm -f rcs-broker-probe >/dev/null 2>&1
    rm -rf "$WORK"
}
trap cleanup EXIT

say()  { printf '\n\033[1m== %s\033[0m\n' "$*"; }
info() { printf '   %s\n' "$*"; }
die()  { printf '\n\033[31mFAIL: %s\033[0m\n' "$*" >&2; exit 1; }

# grep -c prints nothing and exits non-zero when the file is missing, which
# then poisons every arithmetic comparison downstream.
count() { grep -c "$1" "$2" 2>/dev/null || true; }
count0() { local n; n="$(count "$1" "$2")"; echo "${n:-0}"; }

# ── preconditions ────────────────────────────────────────────────────────────
# Every failure below produced a confusing error further downstream, so they
# are checked up front with a message naming the actual problem.
say "preconditions"

command -v docker >/dev/null || die "docker not found"
docker image inspect "$IMAGE" >/dev/null 2>&1 ||
    die "image $IMAGE is not built. See deploy/rmf/Dockerfile."
[ -x target/debug/vda5050-sim ] ||
    die "target/debug/vda5050-sim missing. Run: cargo build -p vda5050-sim"
[ -x target/debug/gateway ] ||
    die "target/debug/gateway missing. Run: cargo build -p gateway"
[ -f "$MAP" ] || die "map $MAP missing"
[ -f "$SIM_CONFIG" ] || die "simulator config $SIM_CONFIG missing"

# A real publish/subscribe round trip, not a port check and not the broker's
# sys topics: those are commonly disabled, and a bare TCP connect says nothing
# about whether the broker accepts a subscription.
PROBE_TOPIC="rcs/verify-negotiation/probe"
docker run --rm --network host eclipse-mosquitto:2 mosquitto_sub \
    -h "$BROKER_HOST" -p "$BROKER_PORT" -C 1 -W 10 -t "$PROBE_TOPIC" \
    >"$LOG_DIR/probe.out" 2>&1 &
PIDS+=($!)
sleep 1
docker run --rm --network host eclipse-mosquitto:2 mosquitto_pub \
    -h "$BROKER_HOST" -p "$BROKER_PORT" -t "$PROBE_TOPIC" -m ping >/dev/null 2>&1
wait "${PIDS[-1]}" 2>/dev/null
PIDS=("${PIDS[@]:0:$((${#PIDS[@]}-1))}")
grep -q '^ping$' "$LOG_DIR/probe.out" ||
    die "published a probe on $BROKER but never got it back. Is a broker running there?"
info "broker round trip ok at $BROKER"

# ── regenerate the artifacts the adapter and gateway read ────────────────────
# Regenerated every run on purpose: a stale building map is one of the ways
# this silently stops working.
say "regenerating RMF artifacts from LIF"

NAV_GRAPH="$WORK/nav_graph.yaml"
NATIVE_MAP="$WORK/plant-b.l1.json"

cargo run -q -p lif2rmf -- \
    --lif "$MAP" \
    --chargers CHG-01,CHG-02 \
    --out "$NAV_GRAPH" \
    --emit-map "$NATIVE_MAP" \
    >"$LOG_DIR/lif2rmf.log" 2>&1 ||
    die "lif2rmf failed; see $LOG_DIR/lif2rmf.log"

[ -s "$NAV_GRAPH" ] || die "lif2rmf produced no nav graph"
[ -s "$NATIVE_MAP" ] || die "lif2rmf produced no native map"

CHARGERS="$(count0 'is_charger: true' "$NAV_GRAPH")"
[ "$CHARGERS" -ge 1 ] ||
    die "nav graph has no charging waypoint. RMF refuses to add a robot to a fleet without one, and reports it far away from the cause."
info "nav graph: $CHARGERS charging waypoint(s)"

ADAPTER_CONFIG="$WORK/config.yaml"
sed -e "s|^  prefix: .*|  prefix: \"$BROKER\"|" \
    deploy/rmf/fleet_adapter/vda5050_fleet_adapter/config.yaml \
    >"$ADAPTER_CONFIG" || die "could not derive the adapter config"

# ── clear anything left over from a previous run ─────────────────────────────
# A gateway still holding 127.0.0.1:8090 makes this one die with AddrInUse,
# and every mission submission then fails silently — which looks exactly like
# "the robots never conflicted".
say "clearing leftovers from any previous run"

pkill -f 'target/debug/gateway' 2>/dev/null
pkill -f 'target/debug/vda5050-sim' 2>/dev/null
docker ps -q --filter ancestor="$IMAGE" | xargs -r docker kill >/dev/null 2>&1
sleep 1

# ── start the simulator ──────────────────────────────────────────────────────
# Without it the adapter has no robot state, registers no participant, and the
# schedule node has nothing to arbitrate. That failure is indistinguishable
# from "no conflict", which is why it is checked explicitly below.
say "starting simulator"

SIM_CONFIG_LIF="$WORK/sim.yaml"
sed -e "s|map: .*|map: $NATIVE_MAP|" "$SIM_CONFIG" >"$SIM_CONFIG_LIF"

target/debug/vda5050-sim --config "$SIM_CONFIG_LIF" \
    >"$LOG_DIR/simulator.log" 2>&1 &
PIDS+=($!)

# ── start the gateway, to issue missions ─────────────────────────────────────
say "starting gateway"

target/debug/gateway \
    --broker-host "$BROKER_HOST" --broker-port "$BROKER_PORT" \
    --http-addr 127.0.0.1:8090 \
    --map "$NATIVE_MAP" \
    --api-token "$GATEWAY_TOKEN" \
    --adapter-manufacturer acme \
    >"$LOG_DIR/gateway.log" 2>&1 &
PIDS+=($!)

# ── start the schedule node and the fleet adapter ────────────────────────────
# Both in one container: DDS discovery is per-network, and --network host is
# the only way to get the adapter's DDS traffic to the schedule node.
#
# Inputs come from /work, logs go to /logs, so a run's logs are readable from
# the host at $LOG_DIR rather than disappearing with the container.
say "starting schedule node and fleet adapter"

# The container-side script is written to a file rather than inlined: nesting
# it inside a single-quoted docker argument needs quote-escaping tricks that
# silently mangle the script, and a mangled script here looks like an RMF
# failure rather than a quoting bug.
INNER="$WORK/inner.sh"
cat >"$INNER" <<INNER_EOF
#!/usr/bin/env bash
# No `set -u`: sourcing ROS's setup.bash touches variables that are not set,
# and under `set -u` that aborts the script before anything starts.
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=/rmf/fleet_adapter:\$PYTHONPATH
# The image ships a pip-installed copy of the adapter that shadows the mounted
# source and is an older revision.
rm -rf /usr/local/lib/python3.12/dist-packages/vda5050_fleet_adapter

ros2 run rmf_traffic_ros2 rmf_traffic_schedule > /logs/scheduler.log 2>&1 &
SCHED=\$!

# The schedule node has to be advertising before the adapter tries to register,
# or the adapter's mirror times out and retries.
sleep 12

python3 -u -m vda5050_fleet_adapter.fleet_adapter \
    -c /work/config.yaml \
    -n /work/nav_graph.yaml \
    > /logs/adapter.log 2>&1 &
ADAPTER=\$!

# Recorded continuously rather than with --once, which would miss a conclusion
# that arrived before the echo started.
ros2 topic echo /rmf_traffic/negotiation_conclusion \
    > /logs/conclusion.raw 2>&1 &
ECHO=\$!

sleep ${CONCLUSION_TIMEOUT_S}

kill \$ECHO \$ADAPTER \$SCHED 2>/dev/null
wait 2>/dev/null
INNER_EOF

# Inputs come from /work, logs go to /logs, so a run's logs are readable from
# the host at $LOG_DIR rather than disappearing with the container.
docker run --rm --network host \
    -v "$ROOT/deploy/rmf:/rmf:ro" \
    -v "$WORK:/work" \
    -v "$LOG_DIR:/logs" \
    -e ROS_DOMAIN_ID="$DOMAIN" \
    "$IMAGE" bash /work/inner.sh >"$LOG_DIR/docker.log" 2>&1 &
PIDS+=($!)

# ── wait for participants ────────────────────────────────────────────────────
# The adapter registers a participant only once it has seen that robot's state
# on MQTT. Checking this separately turns "no conclusion" into a diagnosis.
say "waiting for participants"

PARTICIPANTS=0
for _ in $(seq 1 $((ADAPTER_WARMUP_S / 2))); do
    PARTICIPANTS="$(count0 'Registered participant' "$LOG_DIR/scheduler.log")"
    [ "$PARTICIPANTS" -ge 2 ] && break
    sleep 2
done

if [ "$PARTICIPANTS" -lt 2 ]; then
    info "participants registered: $PARTICIPANTS (expected 2)"
    info "--- adapter log (last 25) ---"
    tail -25 "$LOG_DIR/adapter.log" 2>/dev/null
    die "the adapter registered no participants. Either it never saw robot state, or RMF refused the robot. Look for 'Unable to find a robot transform' and 'Unable to find nearest charging waypoint'."
fi
info "participants registered: $PARTICIPANTS"

# ── give the adapter something to negotiate about ────────────────────────────
# The adapter dispatches parking tasks on its own, which is enough to produce
# conflicting itineraries. Missions are sent as well so the conflict does not
# depend on the parking path alone.
say "issuing missions"

for robot in AMR-001 AMR-002; do
    # The response is shown rather than discarded: a refused mission is the
    # difference between "no conflict" and "no missions at all", and those
    # need opposite fixes.
    RESPONSE="$(curl -s --noproxy '*' -m 5 -X POST "http://127.0.0.1:8090/api/orders" \
        -H "Authorization: Bearer $GATEWAY_TOKEN" \
        -H "Content-Type: application/json" \
        -d "{\"manufacturer\":\"acme\",\"serialNumber\":\"$robot\",\"orderId\":\"verify-$robot\",\"nodes\":[{\"nodeId\":\"AS-1W\",\"sequenceId\":0,\"released\":true,\"actions\":[]},{\"nodeId\":\"AS-1E\",\"sequenceId\":1,\"released\":true,\"actions\":[]}]}")"
    if echo "$RESPONSE" | grep -q '"status":"published"'; then
        info "mission to $robot published"
    else
        info "mission to $robot REFUSED: $(echo "$RESPONSE" | head -c 200)"
    fi
done

# ── wait for the conclusion ──────────────────────────────────────────────────
say "waiting for a negotiation conclusion (${CONCLUSION_TIMEOUT_S}s)"

RESOLVED=0
for _ in $(seq 1 "$CONCLUSION_TIMEOUT_S"); do
    if grep -q 'resolved: true' "$LOG_DIR/conclusion.raw" 2>/dev/null; then
        RESOLVED=1
        break
    fi
    sleep 1
done

# ── report ───────────────────────────────────────────────────────────────────
# The assertable part has passed by the time we get here. What follows is a
# report on the part that cannot be asserted yet, plus the --strict escape
# hatch for when the dispatch path exists.
say "result"

NEGOTIATIONS="$(count0 'Require negotiation' "$LOG_DIR/scheduler.log")"
CONCLUSIONS="$(count0 'conflict_version' "$LOG_DIR/conclusion.raw")"
RESOLVED_TEXT="$(grep -A6 'conflict_version' "$LOG_DIR/conclusion.raw" 2>/dev/null | head -8)"

info "participants registered     : $PARTICIPANTS"
info "negotiations started        : $NEGOTIATIONS"
info "conclusions observed        : $CONCLUSIONS"

if [ "$RESOLVED" -eq 1 ]; then
    info "conclusion                  : resolved: true"
    echo "$RESOLVED_TEXT" | sed 's/^/    /'
    say "PASS"
    info "logs kept in $LOG_DIR"
    trap - EXIT
    exit 0
fi

info "conclusion                  : none"

if [ "$STRICT" -eq 1 ]; then
    say "FAIL (--strict)"
    info "no concluded negotiation. With --strict this is a failure; without it"
    info "the run above is a report, because reaching a conclusion on demand"
    info "needs the dispatch path this adapter does not have yet."
    if [ "$NEGOTIATIONS" -eq 0 ]; then
        info ""
        info "no negotiation started at all. The adapter registered participants"
        info "but nothing it planned overlapped anything else. Check the adapter"
        info "log for a refused transform or a mapId that does not match the nav"
        info "graph level name."
    fi
    info ""
    info "--- adapter log (last 30) ---"
    tail -30 "$LOG_DIR/adapter.log" 2>/dev/null
    info ""
    info "full logs in $LOG_DIR"
    exit 1
fi

say "PASS (assertable checks only)"
info ""
info "A negotiation did not conclude, which is expected rather than a failure:"
info "this adapter has no dispatch path, so itineraries come only from its own"
info "parking task and only sometimes overlap a robot's driving. Run with"
info "--strict once that path exists."
info ""
info "To verify the arbitration mechanism deterministically, run the probe:"
info "    python3 deploy/rmf/traffic_probe.py L1"
info "full logs in $LOG_DIR"
exit 1
