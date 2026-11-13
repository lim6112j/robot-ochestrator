#!/usr/bin/env python3
"""
Probe for RMF traffic arbitration on our warehouse nav graph.

STATUS: incomplete. See "What this does not yet prove" below.

It runs the pieces that decide who goes first through a shared aisle:

  building_map_server  → publishes our nav graph on /map
  rmf_traffic_schedule → arbitrates participant itineraries
  this script          → registers two participants, submits conflicting
                         itineraries, and reports who was admitted when.

The participants are stand-ins for two robots from different manufacturers.
What is being verified is that RMF's schedule accepts *our* map and *our*
robot identities and actually serialises them, not that a real robot moved.

What this does not yet prove
----------------------------
No negotiation is ever started, and the reason is found: the schedule node
does not subscribe to /map. Verified with `ros2 topic info /map`, which
reports `Subscription count: 0` even after 25s of discovery time. So the
schedule runs without a nav graph, and without one there is nothing to
arbitrate.

Everything upstream of that works — registration, itinerary submission,
live participant state and progress reporting all reach the node — which is
what narrows the gap to the nav-graph link.

What is missing: the component that turns a `.building.yaml` into a schedule
nav-graph patch. In RMF that is `rmf_traffic_editor`, but this image ships
only `rmf_traffic_editor_assets` — the Qt editor binary is not installed, and
a headless container cannot run it. Supplying the patch needs either a source
build of rmf_traffic_editor or composing `SchedulePatch` messages directly
through RMF's API.

Not verified either, and not claimed: the fleet adapter. Translating an RMF
itinerary into a VDA 5050 order is `deploy/rmf/fleet_adapter`'s job and is
separate work.
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
    w = TrajectoryWaypoint()
    w.time = t
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
    traj.waypoints = [waypoint(t, x, y) for (t, x, y) in points]
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
    map_name = args[0] if args else 'plant-b/l1'

    node = ArbitrationProbe()
    node.wait_for_service()
    print(f'schedule node is up, map={map_name}')

    # Two participants, two manufacturers, same narrow aisle.
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

    # Both robots want the same stretch of aisle, at the same time. The first
    # should be admitted; the second must be told to wait.
    #
    # The waypoints follow lane centrelines from the nav graph: AS-1W(0,6) →
    # AS-1M(18,6) → AS-1E(36,6) is one lane of aisle 1, and both robots are put
    # on it travelling opposite ways. Off-graph coordinates are silently
    # ignored by the schedule, which is why an earlier diagonal route produced
    # no negotiation at all.
    aisle = [(0.0, 6.0, 0.0), (9.0, 6.0, 10.0), (18.0, 6.0, 20.0), (27.0, 6.0, 30.0), (36.0, 6.0, 40.0)]
    for index, (name, points) in enumerate([
        ('acme:AMR-002', aisle),
        # Same lane, opposite direction, same time window.
        ('sweepco:SCR-001', list(reversed([(t, x, y) for (x, y, t) in aisle]))),
    ]):
        iset = ItinerarySet()
        iset.participant = index
        iset.plan = 0
        iset.itinerary = [route(map_name, points, 0)]
        iset.storage_base = 0
        iset.itinerary_version = 0
        node.pub.publish(iset)
        print(f'  itinerary submitted for {name} (participant {index})')

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
    )

    node.notices = []
    node.conclusions = []
    node.proposals = []
    node.refusals = []
    node.rejections = []
    node.acks = []

    def make_handler(bucket, label):
        def handler(msg):
            bucket.append(msg)
        return handler

    for topic, msg_type, bucket, label in [
        ('/rmf_traffic/negotiation_notice', NegotiationNotice, node.notices, 'notice'),
        ('/rmf_traffic/negotiation_proposal', NegotiationProposal, node.proposals, 'proposal'),
        ('/rmf_traffic/negotiation_ack', NegotiationAck, node.acks, 'ack'),
        ('/rmf_traffic/negotiation_refusal', NegotiationRefusal, node.refusals, 'refusal'),
        ('/rmf_traffic/negotiation_rejection', NegotiationRejection, node.rejections, 'rejection'),
        ('/rmf_traffic/negotiation_conclusion', NegotiationConclusion, node.conclusions, 'conclusion'),
    ]:
        node.create_subscription(msg_type, topic, make_handler(bucket, label), 10)

    # Give the schedule time to negotiate.
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
    for index, desc in enumerate(node.registered):
        p = Participant()
        p.id = index
        p.description = desc
        participants.participants.append(p)
    node.participants_pub.publish(participants)
    print('  live participants published')

    for index in range(len(node.registered)):
        reached = ItineraryReached()
        reached.participant = index
        reached.plan = 0
        reached.reached_checkpoints = []
        reached.progress_version = 1
        node.reached_pub.publish(reached)

        # A small delay tells the schedule the robot is not yet at its target,
        # which is what makes the two itineraries contend for the same lane.
        delay = ItineraryDelay()
        delay.participant = index
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
