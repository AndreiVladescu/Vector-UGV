# ROS 2 Jazzy image for the robot (CM5 / Pi 5, arm64) and the desktop.
#   docker build -f software/docker/Dockerfile -t vector .              (from the repo root)
#   docker build -f software/docker/Dockerfile --target build -t vector:build .   (with the tests)
# No Gazebo or rviz in here: those stay on the desktop.

FROM ros:jazzy-ros-base AS base
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-controller-manager ros-jazzy-forward-command-controller ros-jazzy-gpio-controllers \
      ros-jazzy-joint-state-broadcaster ros-jazzy-robot-state-publisher ros-jazzy-xacro \
      ros-jazzy-foxglove-bridge ros-jazzy-robot-localization ros-jazzy-nmea-navsat-driver ros-jazzy-vision-msgs \
      python3-can python3-yaml python3-serial can-utils stm32flash \
    && rm -rf /var/lib/apt/lists/*

FROM base AS build
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-launch-testing-ament-cmake ros-jazzy-launch-testing-ros ros-jazzy-ament-cmake-pytest \
      ros-jazzy-ament-cmake-gtest python3-pytest iproute2 \
    && rm -rf /var/lib/apt/lists/*
# the repo layout matters: vector_hw builds sim_legs from ../../../../firmware
COPY firmware/common /ws/firmware/common
COPY firmware/leg-node /ws/firmware/leg-node
COPY firmware/power-node /ws/firmware/power-node
COPY protocol /ws/protocol
COPY docs/servos.md /ws/docs/servos.md
COPY software/ros2_ws/src /ws/software/ros2_ws/src
WORKDIR /ws/software/ros2_ws
# one package at a time, two jobs: 2 GB of RAM is enough for that
RUN . /opt/ros/jazzy/setup.sh && MAKEFLAGS=-j2 colcon build --parallel-workers 1 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion- 2>&1 | tail -5

FROM base AS runtime
COPY --from=build /ws/software/ros2_ws/install /ws/install
COPY --from=build /ws/docs/servos.md /ws/docs/servos.md
COPY software/docker/entrypoint.sh /entrypoint.sh
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
