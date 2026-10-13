# One image for both machines: the Mac (Apple silicon) and the Raspberry Pi 5 are both arm64,
# so the Pi runs the same packages the Mac was tested with.
#   target "robot": what runs on the Pi.
#   target "dev":   robot + simulation tools, used on the Mac.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS robot
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-foxglove-bridge \
      python3-colcon-common-extensions \
      python3-pytest \
      python3-serial \
      python3-yaml \
    && rm -rf /var/lib/apt/lists/*
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /apollo/ros2_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

FROM robot AS dev
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-webots-ros2 \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*
