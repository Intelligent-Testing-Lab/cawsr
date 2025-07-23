FROM osrf/ros:humble-desktop

# TODO when deploying
# create new user for security
# for dev, this is fine for now

# install dependencies and add github ssh key to known hosts
RUN apt -y update && \
    apt install --no-install-recommends -y libpng16-16 libtiff5 libjpeg8 build-essential curl wget git libxerces-c-dev && \
    mkdir /root/.ssh && \
    touch /root/.ssh/known_hosts && \
    ssh-keyscan github.com >> /root/.ssh/known_hosts
 
# change to be supplied via env variable
COPY docker/id_rsa /root/.ssh 

# clone repo
RUN git clone git@github.com:Intelligent-Testing-Lab/autoware_scenario_runner.git 

WORKDIR /autoware_scenario_runner

RUN mkdir /ros_workspace &&  \
    mv docker/autoware_msgs.tar /ros_workspace/ && \
    tar -xvf /ros_workspace/autoware_msgs.tar && \
    rm -rf /ros_workspace/autoware_msgs.tar && \
    source /opt/ros/humble/setup.bash && \
    cd /ros_workspace/ && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

ENV AUTOWARE_MSG_PKG="/ros_workspace/install/setup.bash"
ENV ROS_PKG="/opt/ros/${ROS_DISTRO}/setup.bash"

# install pip requirements and carla
RUN python3 -m pip install -r requirements.txt && \
    mv docker/PythonAPI.tar ./ && \
    tar -xvf PythonAPI.tar && \
    rm -rf PythonAPI.tar

ENV CARLA_API_ROOT="/autoware_scenario_runner/PythonAPI"
ENV PYTHONPATH="${PYTHONPATH}:${CARLA_API_ROOT}/carla/agents:${CARLA_API_ROOT}/carla"

ENTRYPOINT [ "/autoware_scenario_runner/entrypoint.sh" ] 
