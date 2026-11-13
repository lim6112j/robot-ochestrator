#!/usr/bin/env python3
# Copyright 2026 Robot Control System
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Publish RMF door states and grant door requests.

This is what makes a `DOOR-*` node in the VDA 5050 map a resource robots must
reserve. `lif2rmf` already emits the door and the lanes that cross it carry
`door_name`; without this adapter RMF has a door that is never open and never
answers a request, so every mission crossing one would stall.

The non-obvious part is what `MODE_OPEN` means. It does not mean "physically
open" — it means "available", and this adapter decides. Publishing
`MODE_CLOSED` forever deadlocks; publishing `MODE_OPEN` unconditionally
defeats the reservation. So a door opens when asked and stays open for a
hold-off long enough for the requesting robot to clear it, which is the
behaviour RMF's own examples use.
"""

import sys
import time

import rclpy
from rclpy.node import Node

from rmf_door_msgs.msg import DoorMode, DoorRequest, DoorState

import yaml


class Vda5050DoorAdapter(Node):
    def __init__(self, config_path):
        super().__init__('vda5050_door_adapter')

        with open(config_path) as f:
            config = yaml.safe_load(f)

        self.doors = config.get('doors', [])
        self.publish_period = float(config.get('publish_period_s', 1.0))

        # door name -> mode. Doors start closed: nothing is available until
        # something asks, which is the honest initial state.
        self._modes = {d['name']: DoorMode.MODE_CLOSED for d in self.doors}
        # door name -> unix time when the hold-off expires.
        self._hold_until = {d['name']: 0.0 for d in self.doors}
        self._hold_off = {
            d['name']: float(d.get('hold_off_s', 6.0)) for d in self.doors
        }

        self.state_pub = self.create_publisher(DoorState, 'door_states', 10)
        self.request_sub = self.create_subscription(
            DoorRequest, 'door_requests', self._on_request, 10
        )

        period = max(0.1, self.publish_period)
        self.timer = self.create_timer(period, self._publish_states)

        self.get_logger().info(
            f'door adapter up, publishing {len(self.doors)} doors '
            f'every {period}s'
        )

    def _on_request(self, request: DoorRequest):
        # A request for a door we do not know is ignored rather than opened:
        # granting it would let a robot through a door that does not exist.
        if request.door_name not in self._modes:
            self.get_logger().warn(
                f'ignoring request for unknown door {request.door_name}'
            )
            return

        now = time.time()
        self._hold_until[request.door_name] = now + self._hold_off[request.door_name]
        self._modes[request.door_name] = DoorMode.MODE_OPEN
        self.get_logger().info(
            f'{request.door_name} opened for {request.requester_id} '
            f'for {self._hold_off[request.door_name]:.0f}s'
        )

    def _publish_states(self):
        now = time.time()
        for name, mode in self._modes.items():
            if mode == DoorMode.MODE_OPEN and now > self._hold_until[name]:
                # Hold-off expired: close again so the next robot has to ask.
                self._modes[name] = DoorMode.MODE_CLOSED

            state = DoorState()
            state.door_name = name
            state.current_mode.value = self._modes[name]
            stamp = self.get_clock().now().to_msg()
            state.door_time = stamp
            self.state_pub.publish(state)


def main(argv=None):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv if argv else sys.argv)

    import argparse

    parser = argparse.ArgumentParser(prog='door_adapter')
    parser.add_argument('-c', '--config', required=True, help='config yaml')
    parsed, _ = parser.parse_known_args(args_without_ros[1:])

    node = Vda5050DoorAdapter(parsed.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
