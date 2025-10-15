FROM osrf/ros:humble-desktop

RUN apt -y update && \
    apt install --no-install-recommends -y libpng16-16 libtiff5 libjpeg8 build-essential curl wget git libxerces-c-dev python3-pip vim

# clone repo
COPY . /autoware_scenario_runner

WORKDIR /autoware_scenario_runner

SHELL [ "/bin/bash", "-c" ]

RUN mkdir /ros_workspace/ &&  \
    cd /ros_workspace/ && \
    mv /autoware_scenario_runner/docker/autoware_msgs.tar.xz /ros_workspace/ && \
    tar -xvf /ros_workspace/autoware_msgs.tar.xz && \
    rm -rf /ros_workspace/autoware_msgs.tar.xz && \
    source /opt/ros/humble/setup.bash && \
    apt install -y ros-humble-rmw-cyclonedds-cpp ros-humble-tf-transformations && \
    rosdep install -i --from-path /ros_workspace/autoware_msgs/src --rosdistro humble -y &&  cd autoware_msgs && \
    colcon build

ENV AUTOWARE_MSG_PKG="/ros_workspace/autoware_msgs/install/setup.bash"
ENV ROS_PKG="/opt/ros/${ROS_DISTRO}/setup.bash"

# install docker inside container
RUN apt-get update -y && \
    apt-get install ca-certificates curl -y && \
    install -m 0755 -d /etc/apt/keyrings && \
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc && \
    chmod a+r /etc/apt/keyrings/docker.asc && \
    echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
        $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
        tee /etc/apt/sources.list.d/docker.list > /dev/null && \
    apt-get update -y && \
    apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y

# install pip requirements and carla
# NETWORKX has issues with collections.abc
# switch to different versions (python 3.9+)
RUN python3 -m pip install -r requirements.txt && \
    mv docker/PythonAPI.tar ./ && \
    tar -xvf PythonAPI.tar && \
    rm -rf PythonAPI.tar && \
    mkdir logs

# update CYCLONE DDS Config for ROS
RUN mkdir /cyclonedds && \
    mv /autoware_scenario_runner/docker/cyclonedds.xml /cyclonedds/ && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///cyclonedds/cyclonedds.xml" >> ~/.bashrc && \
    echo "alias rossrc='source ${AUTOWARE_MSG_PKG} && source ${ROS_PKG} && echo Sourced'" >> ~/.bashrc && \
    source ~/.bashrc

ENV CARLA_API_ROOT="/autoware_scenario_runner/PythonAPI"
ENV PYTHONPATH="${PYTHONPATH}:${CARLA_API_ROOT}/carla/agents:${CARLA_API_ROOT}/carla"

ENTRYPOINT [ "/autoware_scenario_runner/entrypoint.sh" ]
