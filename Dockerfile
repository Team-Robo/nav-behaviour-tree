FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    ros-humble-foxglove-bridge \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-robot-localization \
    ros-humble-tf2-tools \
    ros-humble-rqt-tf-tree \
    ros-humble-diagnostic-updater \
    build-essential \
    cmake \
    wget \
    nano \
    git \
    && rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=burger

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /etc/bash.bashrc

WORKDIR /ros2_ws

CMD ["bash"]
