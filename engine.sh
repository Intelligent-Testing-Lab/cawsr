#!/bin/bash

# start the autoware container with docker compose
docker compose -f docker-compose-autoware.yaml up -d

# enter the container and start autoware
docker exec -it autoware bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic pub -1 /autoware/restart autoware_cawsr_msgs/msg/AutowareRestart "{carla_map: 'Town01', ego_name: 'ego_vehicle'}""
