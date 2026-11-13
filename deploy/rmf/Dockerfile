# ROS 2 Jazzy + Open-RMF on Ubuntu 24.04.
#
# ADR-004: RMF ships binaries for Jazzy only, so the distro is decided by RMF
# availability rather than by support window (Lyrical lasts longer but has no
# RMF binaries). The RMF boundary language is Python (fleet_adapter_template),
# which is why this image ships Python rather than Rust.
#
# RMF binaries live in the standard ROS 2 apt repository, so no extra source
# list or keyring is needed — `ros-jazzy-rmf-dev` is the official aggregate.
#
#   docker build -f deploy/rmf/Dockerfile -t rcs-rmf:jazzy deploy/rmf
#   docker run --rm -it --network host rcs-rmf:jazzy ros2 topic list
FROM ros:jazzy-ros-base

# ros-dev-tools brings rosdep and colcon; rmf-dev aggregates rmf_core,
# rmf_traffic, rmf_task and the fleet adapter packages.
RUN apt-get update -qq \
    && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        ros-jazzy-rmf-dev \
        ros-jazzy-rmf-building-map-tools \
        ros-jazzy-rmf-door-msgs \
        ros-jazzy-rmf-dispenser-msgs \
        ros-jazzy-rmf-lift-msgs \
        python3-colcon-common-extensions \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# The fleet adapter talks VDA 5050 over MQTT. `rmf_fleet_adapter_python` is
# installed from apt, but the template's own client library is not a
# dependency, so bring it in here.
RUN pip3 install --break-system-packages --no-cache-dir paho-mqtt

# rosdep needs its cache before it can resolve dependencies of a source
# workspace; seed it so a later `rosdep install` does not need the network.
RUN rosdep init 2>/dev/null; rosdep update --rosdistro jazzy 2>&1 | tail -1

# The VDA 5050 door adapter: publishes door_states and grants door_requests
# for the doors lif2rmf models out of the map. Installed the same way as the
# fleet adapter (see the note below about why pip and not colcon).
COPY door_adapter /rmf_ws/src/vda5050_door_adapter
RUN pip3 install --break-system-packages --no-cache-dir /rmf_ws/src/vda5050_door_adapter \
    && printf '#!/bin/bash\nsource /opt/ros/jazzy/setup.bash\nexec python3 -m vda5050_door_adapter.door_adapter "$@"\n' \
        > /usr/local/bin/vda5050-door-adapter \
    && chmod +x /usr/local/bin/vda5050-door-adapter

# The VDA 5050 fleet adapter (ADR-004).
#
# The upstream template is an ament_python package, but making colcon build it
# requires either a CMakeLists shim or a `--meta` description, and neither
# registers the ament index entry `ros2 run` needs without extra ceremony.
# A plain `pip install` of the package directory is the same install with one
# step instead of four, so that is what this does.
COPY fleet_adapter /rmf_ws/src/vda5050_fleet_adapter
RUN pip3 install --break-system-packages --no-cache-dir /rmf_ws/src/vda5050_fleet_adapter \
    && printf '#!/bin/bash\nsource /opt/ros/jazzy/setup.bash\nexec python3 -m vda5050_fleet_adapter.fleet_adapter "$@"\n' \
        > /usr/local/bin/vda5050-fleet-adapter \
    && chmod +x /usr/local/bin/vda5050-fleet-adapter

# The dev container is interactive: source ROS for every shell so `ros2 ...`
# works without manual setup.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /rmf
CMD ["/bin/bash"]
