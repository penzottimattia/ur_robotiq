FROM ros:humble

RUN mkdir -p /ws/src
WORKDIR /ws/src

RUN apt update && apt install -y \
    openssh-client \
    python3-pip \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

RUN pip install dynamixel_sdk

RUN git clone -b $ROS_DISTRO https://github.com/penzottimattia/ur_robotiq.git
RUN git clone -b $ROS_DISTRO https://github.com/penzottimattia/ros2_robotiq_gripper.git
RUN git clone -b ros2 https://github.com/penzottimattia/serial.git

RUN apt update; \
    rosdep update; \
    rosdep install --from-paths . --ignore-src -r -y

RUN cp /opt/ros/$ROS_DISTRO/share/ur_description/config/ur3e/default_kinematics.yaml /ws/src/ur_robotiq/config/left_ur_calibration.yaml
RUN cp /opt/ros/$ROS_DISTRO/share/ur_description/config/ur3e/default_kinematics.yaml /ws/src/ur_robotiq/config/right_ur_calibration.yaml

WORKDIR /ws
RUN bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --install-base /opt/ros/$ROS_DISTRO/ur_robotiq'

WORKDIR /
RUN rm -rf /ws

RUN apt update && apt install -y \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN ln -s /opt/ros/$ROS_DISTRO/ur_robotiq/ur_robotiq/lib/python3.10/site-packages/ur_robotiq/gello_publisher.py /gello_publisher.py
