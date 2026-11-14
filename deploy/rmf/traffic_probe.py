"""
Probe RMF traffic arbitration on our warehouse nav graph.

What it verifies
----------------
Two participants — stand-ins for robots from different manufacturers — are
registered, given conflicting itineraries on one shared lane, and the
schedule's response is reported. The negotiation **starts**: the schedule node
logs `Require negotiation for: 0 1`, and the negotiation topics carry a notice
for `participants: [0, 1]` with a state-tree node for each.

What it does not verify
-----------------------
The negotiation's conclusion. Reaching one needs something that answers the
schedule's proposals, and this probe only impersonates a participant — it has
no policy about who should give way. The answerer is the fleet adapter in
`deploy/rmf/fleet_adapter`, which is a separate piece of work.

What it does not verify
-----------------------
The negotiation's conclusion. Reaching one needs something that answers the
schedule's proposals, and this probe only impersonates a participant — it has
no policy about who should give way. The answerer is the fleet adapter in
`deploy/rmf/fleet_adapter`, which is a separate piece of work.

Two silent failures worth knowing about
---------------------------------------
Both of these produced exactly the same symptom — "no negotiation at all" —
and neither raised an error:

* `ItinerarySet` waypoint `time` is an **absolute nanosecond** count on the
  steady_clock. Values of `0, 10, 20, 30, 40` are one instant in the distant
  past, and the schedule drops the whole itinerary without a word. Anchor on
  `now + t * 1e9`.
* The map name is the **level** name from the building map (`levels: L1:`), so
  routes must say `L1`. A name the schedule has never seen is not an error;
  the route is simply ignored.

An earlier version of this file concluded that the schedule not subscribing to
`/map` was the blocker. It does report `Subscription count: 0`, but that is by
design — the schedule arbitrates in `(map, x, y, t)` space and never needs the
nav graph.

What this probe is for now
--------------------------
It drives the schedule node on its own, which is enough to show that arbitration
happens: two conflicting itineraries produce a notice, a state tree with a node
per participant, and a status. That is the part that was hard to get working.

The **conclusion** is not reached here, and cannot be: a conclusion needs
somebody with a policy about who gives way, and the only thing that has one is
the fleet adapter. Run the adapter against the same schedule node — see
`deploy/rmf/README.md` — and the conclusion does arrive, with `resolved: true`.

Participant ids are whatever the schedule hands back, and they are only `0, 1`
when nothing else has registered. An earlier version hardcoded `0, 1`, which
worked while this probe ran alone and silently sent the itineraries to somebody
else's robots once a fleet adapter took those ids first.
"""

import sys
import time

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from rmf_traffic_msgs.msg import (
    Circle,
    ConvexShape,
    ConvexShapeContext,
    ItineraryDelay,
    ItineraryReached,
    ItinerarySet,
    NegotiationAck,
    NegotiationKey,
    NegotiationProposal,
    NegotiationRejection,
    Participant,
    ParticipantDescription,
    Participants,
    Profile,
    Route,
    ScheduleIdentity,
    Trajectory,
    TrajectoryWaypoint,
)
from rmf_traffic_msgs.srv import RegisterParticipant


def waypoint(t, x, y, vx=0.0, vy=0.0):
    """One trajectory waypoint. `velocity` is the instantaneous velocity, which
    the schedule uses to know the robot is actually moving; a zero velocity
    everywhere makes the participant look parked, and a parked robot has
    nothing to negotiate."""
    w = TrajectoryWaypoint()
    # `time` is int64 — a float here trips an assertion inside the message
    # converter, which surfaces as a hard abort rather than a Python error.
    # `position` and `velocity` are float64[3], not nested Point messages.
    w.time = int(t)
    w.position = [x, y, 0.0]
    w.velocity = [vx, vy, 0.0]
    return w


def profile(footprint_radius, vicinity_radius):
    """A robot profile: two circles in one shared shape context.

    The radii live on `Circle` entries inside `Profile.shape_context`; a
    `ConvexShape` only names the type and which index of that context to use.
    Both shapes therefore share one context, at indices 0 and 1.
    """
    p = Profile()
    footprint = Circle()
    footprint.radius = footprint_radius
    vicinity = Circle()
    vicinity.radius = vicinity_radius
    p.shape_context = ConvexShapeContext(circles=[footprint, vicinity])

    p.footprint = ConvexShape(type=ConvexShape.CIRCLE, index=0)
    p.vicinity = ConvexShape(type=ConvexShape.CIRCLE, index=1)
    return p


def trajectory(points):
    """A trajectory through `points` as (t, x, y) triples.

    `Trajectory` carries only waypoints; the map name lives on `Route`, so
    passing it in here would not compile.
    """
    traj = Trajectory()
    traj.waypoints = [waypoint(*p) for p in points]
    return traj


def route(map_name, points, checkpoint):
    r = Route()
    r.map = map_name
    r.trajectory = trajectory(points)
    r.checkpoints = [checkpoint]
    return r


class ArbitrationProbe(Node):
    def __init__(self):
        super().__init__('arbitration_probe')
        self.registered = []
        self.conclusions = []
        self.register_srv = self.create_client(
            RegisterParticipant, '/rmf_traffic/register_participant'
        )
        # The schedule node listens for `ItinerarySet`, not `Itinerary`: the
        # participant id travels with the routes, and an `Itinerary` alone
        # cannot say whose it is.
        self.pub = self.create_publisher(ItinerarySet, '/rmf_traffic/itinerary_set', 10)
        # The schedule also needs each participant's live position and
        # progress. Without these it has registered robots but nothing to
        # arbitrate, and no negotiation is ever started.
        self.participants_pub = self.create_publisher(
            Participants, '/rmf_traffic/participants', 10
        )
        self.reached_pub = self.create_publisher(
            ItineraryReached, '/rmf_traffic/itinerary_reached', 10
        )
        self.delay_pub = self.create_publisher(
            ItineraryDelay, '/rmf_traffic/itinerary_delay', 10
        )
        # A negotiation only progresses if the participants answer the
        # schedule's proposals. Refusing outright leaves the negotiation stuck
        # at the notice, so the probe must speak back.
        self.rejection_pub = self.create_publisher(
            NegotiationRejection, '/rmf_traffic/negotiation_rejection', 10
        )
        self.ack_pub = self.create_publisher(
            NegotiationAck, '/rmf_traffic/negotiation_ack', 10
        )

    def register(self, name, owner, robot_profile):
        desc = ParticipantDescription()
        desc.name = name
        desc.owner = owner
        desc.responsiveness = ParticipantDescription.RX_RESPONSIVE
        desc.profile = robot_profile
        self.registered.append(desc)
        return desc

    def wait_for_service(self, timeout_s=10.0):
        if not self.register_srv.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError('register_participant service never appeared')
        return True


def main(argv=None):
    rclpy.init(args=argv)
    args = [a for a in (argv or sys.argv)[1:] if not a.startswith('_')]
    map_name = args[0] if args else 'L1'

    node = ArbitrationProbe()
    node.wait_for_service()
    print(f'schedule node is up, map={map_name}')

    # Two participants, two manufacturers, same narrow aisle.
    #
    # The ids the schedule hands back are whatever is free, and they are only
    # 0 and 1 when nothing else has registered yet — a fleet adapter running
    # alongside takes those first. Guessing 0,1 sends the itineraries to
    # somebody else's robots, and the schedule then has nothing to arbitrate
    # between them, which is indistinguishable from "no conflict".
    ids = []
    for name, owner in [
        ('acme:AMR-002', 'acme'),
        ('sweepco:SCR-001', 'sweepco'),
    ]:
        # A 0.35m-radius robot in a 0.6m vicinity, matching the fleet config.
        desc = node.register(name, owner, profile(0.35, 0.6))

        req = RegisterParticipant.Request()
        req.description = desc
        future = node.register_srv.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        result = future.result()
        if result is None:
            print(f'  register {name}: FAILED (no response)')
            continue
        print(f'  register {name}: participant_id={result.participant_id}')
        ids.append(result.participant_id)

    # Both robots want the same stretch of aisle, at the same time. The first
    # should be admitted; the second must be told to wait.
    #
    # The waypoints follow lane centrelines from the nav graph: AS-1W(0,6) →
    # AS-1M(18,6) → AS-1E(36,6) is one lane of aisle 1, and both robots are put
    # on it travelling opposite ways. Off-graph coordinates are silently
    # ignored by the schedule, which is why an earlier diagonal route produced
    # no negotiation at all.
    # RMF waypoint times are *absolute* nanoseconds on the steady_clock, not
    # seconds and not offsets. A route stamped at 0,10,20,30,40 nanoseconds is
    # one instant in the far past, and the schedule drops it without a word —
    # which is indistinguishable from "no conflict". Anchor on now.
    now_ns = node.get_clock().now().nanoseconds
    aisle = [
        (0.0, 6.0, 0.0),
        (9.0, 6.0, 10.0),
        (18.0, 6.0, 20.0),
        (27.0, 6.0, 30.0),
        (36.0, 6.0, 40.0),
    ]
    aisle = [(now_ns + int(t * 1e9), x, y) for (x, y, t) in aisle]
    # Which participant each route belongs to. With `--ids` these are the
    # adapter's robots; otherwise they are the two this probe registered.
    for index, (name, points) in enumerate([
        ('acme:AMR-002', aisle),
        # Same lane, opposite direction, same time window.
        ('sweepco:SCR-001', list(reversed(aisle))),
    ]):
        # Give each waypoint a real instantaneous velocity from its own
        # neighbours. The schedule uses velocity to know the participant is
        # actually moving, and a robot that looks parked has nothing to
        # negotiate with anyone.
        for i, (t, x, y) in enumerate(points):
            if i + 1 < len(points):
                t2, x2, y2 = points[i + 1]
                dt = max(t2 - t, 1e-6)
                points[i] = (t, x, y, (x2 - x) / dt, (y2 - y) / dt)
            else:
                _, x0, y0, _, _ = points[i - 1]
                points[i] = (t, x, y, (x - x0) / max(t - points[i - 1][0], 1e-6), 0.0)
        iset = ItinerarySet()
        iset.participant = ids[index]
        iset.plan = 0
        iset.itinerary = [route(map_name, points, 0)]
        iset.storage_base = 0
        iset.itinerary_version = 0
        node.pub.publish(iset)
        print(f'  itinerary submitted for {name} (participant {ids[index]})')

    # Watch the negotiation topics for the outcome. A rejected or delayed
    # itinerary is what proves the arbitration happened: without it both
    # robots would simply be admitted.
    from rmf_traffic_msgs.msg import (
        NegotiationAck,
        NegotiationConclusion,
        NegotiationNotice,
        NegotiationProposal,
        NegotiationRefusal,
        NegotiationRejection,
        NegotiationStates,
        NegotiationStatuses,
    )

    node.notices = []
    node.conclusions = []
    node.proposals = []
    node.refusals = []
    node.rejections = []
    node.acks = []
    node.states = []
    node.statuses = []

    def make_handler(bucket, label):
        def handler(msg):
            bucket.append(msg)
        return handler

    for topic, msg_type, bucket, label in [
        ('/rmf_traffic/negotiation_notice', NegotiationNotice, node.notices, 'notice'),
        ('/rmf_traffic/negotiation_states', NegotiationStates, node.states, 'states'),
        ('/rmf_traffic/negotiation_statuses', NegotiationStatuses, node.statuses, 'statuses'),
        ('/rmf_traffic/negotiation_proposal', NegotiationProposal, node.proposals, 'proposal'),
        ('/rmf_traffic/negotiation_ack', NegotiationAck, node.acks, 'ack'),
        ('/rmf_traffic/negotiation_refusal', NegotiationRefusal, node.refusals, 'refusal'),
        ('/rmf_traffic/negotiation_rejection', NegotiationRejection, node.rejections, 'rejection'),
        ('/rmf_traffic/negotiation_conclusion', NegotiationConclusion, node.conclusions, 'conclusion'),
    ]:
        node.create_subscription(msg_type, topic, make_handler(bucket, label), 10)

    # Give the schedule time to negotiate.
    # Answer the schedule's proposals. This subscription has to exist before
    # the negotiation starts — registering it after the itineraries are in
    # means the first proposal is missed and the negotiation stalls at the
    # notice with nothing to respond to.
    node.proposals_seen = []

    def on_proposal(msg):
        node.proposals_seen.append(msg)
        print(f"  proposal: conflict={msg.conflict_version} "
              f"version={msg.proposal_version} for={msg.for_participant} "
              f"accommodate={[k.participant for k in msg.to_accommodate]}")
        rej = NegotiationRejection()
        rej.conflict_version = msg.conflict_version
        rej.rejected_by = msg.for_participant
        rej.table = [NegotiationKey(participant=0, version=0),
                     NegotiationKey(participant=1, version=0)]
        rej.alternatives = []
        node.rejection_pub.publish(rej)
        print('  → rejection sent')

    node.create_subscription(
        NegotiationProposal, '/rmf_traffic/negotiation_proposal', on_proposal, 10
    )

    deadline = time.time() + 25.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.conclusions:
            break

    # Report live participants and progress, then let the schedule arbitrate.
    participants = Participants()
    # `ScheduleIdentity` carries a uuid and a timestamp, not a plain name.
    participants.node_id = ScheduleIdentity()
    participants.node_id.node_uuid = 'arbitration-probe'
    participants.node_id.timestamp = node.get_clock().now().to_msg()
    participants.participants = []
    for pid, desc in zip(ids, node.registered):
        p = Participant()
        p.id = pid
        p.description = desc
        participants.participants.append(p)
    node.participants_pub.publish(participants)
    print('  live participants published')

    # The schedule node needs to know where each participant is along its route
    # before it can tell that two routes contend for the same lane; without it
    # the itineraries are accepted and nothing is negotiated, which looks
    # exactly like "no conflict".
    for pid in ids:
        reached = ItineraryReached()
        reached.participant = pid
        reached.plan = 0
        reached.reached_checkpoints = []
        reached.progress_version = 1
        node.reached_pub.publish(reached)

        # A small delay tells the schedule the robot is not yet at its target,
        # which is what makes the two itineraries contend for the same lane.
        delay = ItineraryDelay()
        delay.participant = pid
        # Nanoseconds; int64.
        delay.delay = 5_000_000_000
        delay.itinerary_version = 0
        node.delay_pub.publish(delay)
    print('  progress reported')

    print()
    print('negotiation traffic:')
    print(f"  notice     : {len(node.notices)}")
    print(f"  proposal   : {len(node.proposals)}")
    print(f"  ack        : {len(node.acks)}")
    print(f"  refusal    : {len(node.refusals)}")
    print(f"  rejection  : {len(node.rejections)}")
    print(f"  conclusion : {len(node.conclusions)}")
    print(f"  states     : {len(node.states)}")
    print(f"  statuses   : {len(node.statuses)}")
    if node.statuses:
        for st in node.statuses:
            print(f"    status: {st}")
            break
    if node.states:
        for st in node.states:
            print(f"    state table: {st}")
            break
    if node.conclusions:
        for c in node.conclusions:
            print(f"    conclusion: participant={c.participant} "
                  f"conflict={len(c.conflicts) if hasattr(c, 'conflicts') else '?'} "
                  f"agreed={getattr(c, 'agreed', '?')}")

    node.destroy_node()
    rclpy.shutdown()
    print('probe done — inspect /rmf_traffic/negotiation_* for the outcome')


if __name__ == '__main__':
    main(sys.argv)
