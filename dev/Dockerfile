FROM osrf/ros:humble-desktop

RUN apt -y update && \
    apt install --no-install-recommends -y libpng16-16 libtiff5 libjpeg8 build-essential curl wget git libxerces-c-dev python3-pip vim

# Create carla user with same UID/GID as the CARLA simulator container
RUN groupadd -g 1000 carla && \
    useradd -m -u 1000 -g carla -s /bin/bash carla && \
    mkdir /autoware_scenario_runner && \
    mkdir /container_tools

# clone repo
COPY ./docker /autoware_scenario_runner/docker
COPY ./requirements.txt /autoware_scenario_runner/requirements.txt
COPY ./entrypoint.sh /container_tools/entrypoint.sh

WORKDIR /autoware_scenario_runner

SHELL [ "/bin/bash", "-c" ]

RUN mkdir /ros_workspace/ &&  \
    cd /ros_workspace/ && \
    mv /autoware_scenario_runner/docker/autoware_msgs.tar.xz /ros_workspace/ && \
    tar -xvf /ros_workspace/autoware_msgs.tar.xz && \
    rm -rf /ros_workspace/autoware_msgs.tar.xz && \
    source /opt/ros/humble/setup.bash && \
    apt-get update && \
    apt-get install -y ros-humble-rmw-cyclonedds-cpp ros-humble-tf-transformations && \
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

# install pip packages and setup docker entrypoint
RUN python3 -m pip install -r requirements.txt && \
    mkdir -p /carla_api && \
    mv /autoware_scenario_runner/docker/PythonAPI.tar /carla_api/ && \
    tar -xvf /carla_api/PythonAPI.tar -C /carla_api && \
    rm -rf /carla_api/PythonAPI.tar && \
    cd /container_tools && \
    chmod +x /container_tools/entrypoint.sh && \
    mkdir logs

# update CYCLONE DDS Config for ROS
RUN mkdir /cyclonedds && \
    mv /autoware_scenario_runner/docker/cyclonedds_local.xml /cyclonedds/ && \
    mv /autoware_scenario_runner/docker/cyclonedds_distributed.xml /cyclonedds/ && \
    rm -rf /autoware_scenario_runner/docker && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/carla/.bashrc && \
    echo "alias rossrc='source ${AUTOWARE_MSG_PKG} && source ${ROS_PKG} && echo Sourced'" >> /home/carla/.bashrc

ENV CARLA_API_ROOT="/carla_api/PythonAPI"
ENV PYTHONPATH="${PYTHONPATH}:${CARLA_API_ROOT}/carla/agents:${CARLA_API_ROOT}/carla"

# Give carla user ownership of necessary directories
RUN chown -R carla:carla /autoware_scenario_runner && \
    chown -R carla:carla /carla_api && \
    chown -R carla:carla /container_tools && \
    chown -R carla:carla /ros_workspace && \
    chown -R carla:carla /cyclonedds && \
    usermod -aG docker carla

# Switch to carla user
USER carla

ENTRYPOINT [ "/container_tools/entrypoint.sh" ]
