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
    && rm -rf /var/lib/apt/lists/*

# rosdep needs its cache before it can resolve dependencies of a source
# workspace; seed it so a later `rosdep install` does not need the network.
RUN rosdep init 2>/dev/null; rosdep update --rosdistro jazzy 2>&1 | tail -1

# The dev container is interactive: source ROS for every shell so `ros2 ...`
# works without manual setup.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /rmf
CMD ["/bin/bash"]
