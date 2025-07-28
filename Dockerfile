FROM osrf/ros:humble-desktop

RUN apt -y update && \
    apt install --no-install-recommends -y libpng16-16 libtiff5 libjpeg8 build-essential curl wget git libxerces-c-dev python3-pip

# clone repo
COPY . /autoware_scenario_runner

WORKDIR /autoware_scenario_runner

SHELL [ "/bin/bash", "-c" ]

RUN mkdir /ros_workspace/ &&  \
    cd /ros_workspace/ && \
    mv /autoware_scenario_runner/docker/autoware_msgs.tar /ros_workspace/ && \
    tar -xvf /ros_workspace/autoware_msgs.tar && \
    rm -rf /ros_workspace/autoware_msgs.tar && \
    source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path /ros_workspace/src --rosdistro humble -y && \
    colcon build

ENV AUTOWARE_MSG_PKG="/ros_workspace/install/setup.bash"
ENV ROS_PKG="/opt/ros/${ROS_DISTRO}/setup.bash"

# install pip requirements and carla
# NETWORKX has issues with collections.abc
# switch to different versions (python 3.9+)
RUN python3 -m pip install -r requirements.txt && \
    mv docker/PythonAPI.tar ./ && \
    tar -xvf PythonAPI.tar && \
    rm -rf PythonAPI.tar

# update CYCLONE DDS Config for ROS
RUN mkdir /cyclonedds && \
    mv /autoware_scenario_runner/docker/cyclonedds.xml /cyclonedds/ && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///cyclonedds/cyclonedds.xml" >> ~/.bashrc && \
    source ~/.bashrc

ENV CARLA_API_ROOT="/autoware_scenario_runner/PythonAPI"
ENV PYTHONPATH="${PYTHONPATH}:${CARLA_API_ROOT}/carla/agents:${CARLA_API_ROOT}/carla"

ENTRYPOINT [ "/autoware_scenario_runner/entrypoint.sh" ]
