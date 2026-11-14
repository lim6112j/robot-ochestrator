# Copyright 2021 Open Source Robotics Foundation, Inc.
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
The RobotAPI class is a wrapper for API calls to the robot.

Here users are expected to fill up the implementations of functions which
will be used by the RobotCommandHandle. For example, if your robot has a
REST API, you will need to make http request calls to the appropriate
endpoints within these functions.

This implementation talks VDA 5050 over MQTT instead of a REST API, which is
the point of ADR-004: RMF should not care what protocol the robots speak.

The one non-obvious choice: robot state is *subscribed* from MQTT rather than
polled. The same `state` messages the gateway ingests are the source of truth,
so the dashboard and RMF cannot disagree about where a robot is. A polling
implementation would race the gateway and eventually report a stale position.
"""

import json
import time
from typing import Optional

import paho.mqtt.client as mqtt


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, config_yaml):
        fleet = config_yaml['fleet_manager']
        self.prefix = fleet['prefix']
        # `user`/`password` are the upstream REST template's basic-auth fields.
        # A VDA 5050 fleet talks MQTT, where authentication is a broker concern
        # rather than a per-request header, so they are optional here and
        # default to None.
        self.user = fleet.get('user')
        self.password = fleet.get('password')
        self.timeout = 5.0
        self.debug = False

        # VDA 5050 addressing: interface/majorVersion/manufacturer/serialNumber.
        self.interface = fleet.get('interface_name', 'vda5050')
        self.major_version = int(fleet.get('major_version', 3))
        # Which robots this adapter speaks for. Mode A is a direct robot, mode B
        # is a fleet manager fronting several robots; both look the same here
        # because VDA 5050 addresses robots individually either way.
        self.robots = fleet.get('robots', [])
        # VDA 5050 mapIds are the robot vendor's own names and need not match
        # RMF level names — ours are `plant-b/l1` versus `L1`. RMF looks up
        # transforms by level name, so a robot that reports an unmapped mapId
        # is refused with "Unable to find a robot transform" and is never
        # added to the fleet.
        self.map_names = fleet.get('map_names', {})

        self._mqtt = mqtt.Client(
            client_id=f"rmf-adapter-{int(time.time())}",
            transport=fleet.get('transport', 'tcp'),
        )
        if self.user:
            self._mqtt.username_pw_set(self.user, self.password)
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_message = self._on_message

        # Latest state per robot, keyed by "manufacturer:serialNumber".
        self._states = {}
        self._connected = False

    # ── MQTT plumbing ────────────────────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, rc):
        # `v3` in the topic is the major-version level, v-prefixed per the spec.
        # Robots are configured as `manufacturer:serialNumber`, which is two
        # topic levels, so the colon must become a slash — subscribing to
        # `.../acme:AMR-001/state` silently matches nothing.
        prefix = f"{self.interface}/v{self.major_version}"
        for robot in self.robots:
            client.subscribe(f"{prefix}/{robot.replace(':', '/')}/state", qos=1)
        self._connected = rc == 0

    def _on_message(self, client, userdata, msg):
        try:
            state = json.loads(msg.payload)
        except ValueError:
            return
        # topic .../<manufacturer>/<serialNumber>/state
        parts = msg.topic.split('/')
        robot_id = f"{parts[2]}:{parts[3]}"
        self._states[robot_id] = state

    def connect(self):
        # `self.prefix` is `host:port`, so the host has to be split off —
        # passing it whole makes getaddrinfo fail on the colon.
        self._mqtt.connect(self._host(), int(self._port()), keepalive=30)
        self._mqtt.loop_start()

    def _port(self):
        if ':' in self.prefix:
            return self.prefix.rsplit(':', 1)[1]
        return 1883

    def _host(self):
        return self.prefix.rsplit(':', 1)[0] if ':' in self.prefix else self.prefix

    def _publish(self, robot_id: str, topic: str, payload: dict):
        manufacturer, serial = robot_id.split(':', 1)
        full = f"{self.interface}/v{self.major_version}/{manufacturer}/{serial}/{topic}"
        self._mqtt.publish(full, json.dumps(payload), qos=1)

    def _header(self, robot_id: str, order_id: str) -> dict:
        """The VDA 5050 header every message carries.

        `headerId` is an integer, not a string — the spec is explicit and a
        string is rejected. Millisecond timestamps with a trailing Z are
        likewise required, not merely conventional.
        """
        manufacturer, serial = robot_id.split(':', 1)
        return {
            "headerId": int(time.time() * 1000),
            "timestamp": _now_iso(),
            "version": "3.0.0",
            "manufacturer": manufacturer,
            "serialNumber": serial,
            "orderId": order_id,
            "orderUpdateId": 0,
        }

    # ── RobotClientAPI surface ───────────────────────────────────────────────

    def check_connection(self) -> bool:
        """Return True if connection to the robot API server is successful."""
        return self._connected

    def localize(
        self,
        robot_name: str,
        pose,
        map_name: str,
    ) -> bool:
        """Request the robot to localize on target map."""
        # VDA 5050 has no bare "localize" message; `initPosition` is the
        # closest match and is what the spec defines for this purpose.
        self._publish(robot_name, "instantActions", {
            **self._header(robot_name, ""),
            "actions": [{
                "actionType": "initPosition",
                "actionId": f"localize-{int(time.time())}",
                "blockingType": "NONE",
                "actionParameters": [
                    {"key": "x", "value": pose[0]},
                    {"key": "y", "value": pose[1]},
                    {"key": "theta", "value": pose[2]},
                    {"key": "mapId", "value": map_name},
                ],
            }],
        })
        return True

    def navigate(
        self,
        robot_name: str,
        pose,
        map_name: str,
        speed_limit=0.0
    ) -> bool:
        """Request the robot to navigate to pose:[x,y,theta]."""
        # An order with a single node. RMF owns path planning under
        # Full Control, so the adapter asks for a destination, not a route.
        order = {
            **self._header(robot_name, f"rmf-nav-{int(time.time() * 1000)}"),
            "nodes": [{
                "nodeId": f"rmf-goal-{int(time.time() * 1000)}",
                "sequenceId": 0,
                "released": True,
                "nodePosition": {
                    "x": pose[0], "y": pose[1], "theta": pose[2],
                    "mapId": map_name, "localized": True,
                },
                "actions": [],
            }],
            "edges": [],
        }
        self._publish(robot_name, "order", order)
        return True

    def start_activity(
        self,
        robot_name: str,
        activity: str,
        label: str
    ) -> bool:
        """Request the robot to begin a process."""
        # An activity is a node action appended to whatever order is running,
        # which is how VDA 5050 models "do this where you are".
        self._publish(robot_name, "order", {
            **self._header(robot_name, f"rmf-act-{int(time.time() * 1000)}"),
            "nodes": [{
                "nodeId": label,
                "sequenceId": 0,
                "released": True,
                "actions": [{
                    "actionType": activity,
                    "actionId": f"{activity}-{label}",
                    "blockingType": "HARD",
                }],
            }],
            "edges": [],
        })
        return True

    def stop(self, robot_name: str) -> bool:
        """Request the robot to stop."""
        self._publish(robot_name, "instantActions", {
            **self._header(robot_name, ""),
            "actions": [{
                "actionType": "cancelOrder",
                "actionId": f"stop-{int(time.time())}",
                "blockingType": "NONE",
            }],
        })
        return True

    def position(self, robot_name: str) -> Optional[tuple]:
        """Return [x, y, theta] expressed in the robot's coordinate frame."""
        state = self._states.get(robot_name)
        if not state:
            return None
        pos = state.get('mobileRobotPosition')
        if not pos:
            return None
        return (pos['x'], pos['y'], pos.get('theta', 0.0))

    def battery_soc(self, robot_name: str) -> Optional[float]:
        """Return state of charge of robot as a value between 0.0 and 1.0."""
        state = self._states.get(robot_name)
        if not state:
            return None
        power = state.get('powerSupply')
        if not power:
            return None
        # VDA 5050 reports percent; RMF wants a fraction.
        return power.get('stateOfCharge', 0.0) / 100.0

    def get_map_name(self, robot_name: str) -> Optional[str]:
        """Return the name of the map that the robot is currently on."""
        state = self._states.get(robot_name)
        if not state:
            return None
        pos = state.get('mobileRobotPosition')
        if not pos:
            return None
        return self.map_names.get(pos.get('mapId'), pos.get('mapId'))

    def is_command_completed(self) -> bool:
        """Return True if the robot has completed its last command."""
        # A robot is idle when it reports no order and is not driving.
        return all(
            not s.get('orderId') and not s.get('driving', False)
            for s in self._states.values()
        )

    def get_data(self, robot_name: str):
        """Return robot update data for the specified robot."""
        map_name = self.get_map_name(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if not (map_name is None or position is None or battery_soc is None):
            return RobotUpdateData(robot_name, map_name, position, battery_soc)
        return None


def _now_iso() -> str:
    """ISO-8601 with milliseconds, the format VDA 5050 requires."""
    t = time.time()
    millis = int((t % 1) * 1000)
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(t)) + f'.{millis:03d}Z'


class RobotUpdateData:
    """Update data for a single robot."""

    def __init__(self,
                 robot_name: str,
                 map_name: str,
                 position,
                 battery_soc: float):
        self.robot_name = robot_name
        self.map_name = map_name
        self.position = position
        self.battery_soc = battery_soc
