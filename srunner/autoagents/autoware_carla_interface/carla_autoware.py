#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time


from srunner.autoagents.autoware_carla_interface.carla_ros import carla_ros2_interface
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.autoagents.autoware_carla_interface.modules.carla_wrapper import (
    SensorReceivedNoData,
)
from srunner.autoagents.autoware_carla_interface.modules.carla_wrapper import (
    SensorWrapper,
)
from srunner.scenarioconfigs.environment_configuration import EnvironmentConfig


class SensorLoop(object):
    def __init__(self):
        self.start_game_time = None
        self.start_system_time = None
        self.sensor = None
        self.ego_actor = None
        self.running = False
        self.timestamp_last_run = 0.0
        self.timeout = 20.0

    def _stop_loop(self):
        self.running = False

    def _tick_sensor(self, timestamp):
        if self.timestamp_last_run < timestamp.elapsed_seconds and self.running:
            self.timestamp_last_run = timestamp.elapsed_seconds
            try:
                ego_action = self.sensor()
            except SensorReceivedNoData as e:
                raise RuntimeError(e)
            self.ego_actor.apply_control(ego_action)


class InitializeInterface(object):
    def __init__(self, config: EnvironmentConfig, node):
        self.interface = carla_ros2_interface(node)

        self.world = None
        self.sensor_wrapper = None
        self.ego_actor = None
        self.prev_tick_wall_time = 0.0

        self.config = config

    def load_world(self):
        self.ego_actor = CarlaDataProvider.get_actor_by_name(self.config.ego_name)
        self.interface.ego_actor = self.ego_actor  # TODO improve design
        self.interface.physics_control = self.ego_actor.get_physics_control()

        self.sensor_wrapper = SensorWrapper(self.interface)
        self.sensor_wrapper.setup_sensors(self.ego_actor, False)

    def run_bridge(self):
        self.bridge_loop = SensorLoop()
        self.bridge_loop.sensor = self.sensor_wrapper
        self.bridge_loop.ego_actor = self.ego_actor
        self.bridge_loop.start_system_time = time.time()
        self.bridge_loop.start_game_time = GameTime.get_time()
        self.bridge_loop.running = True

    def tick_bridge(self):
        timestamp = None
        world = CarlaDataProvider.get_world()
        if world:
            snapshot = world.get_snapshot()
            if snapshot:
                timestamp = snapshot.timestamp
        if timestamp:
            self.prev_tick_wall_time = time.time()
            self.bridge_loop._tick_sensor(timestamp)

    def _stop_loop(self, sign, frame):
        self.bridge_loop._stop_loop()

    def _cleanup(self):
        """Clean up all CARLA resources in reverse initialization order.

        Ensures cleanup happens even if individual steps fail.
        """
        self._cleanup_sensors()
        self._cleanup_ros_interface()
        self._cleanup_ego_actor()
        self._cleanup_carla_provider()

    def _cleanup_sensors(self):
        """Clean up sensor wrapper, continuing on error."""
        if not self.sensor_wrapper:
            return
        try:
            self.sensor_wrapper.cleanup()
        except Exception as e:
            print(f"Warning: Sensor cleanup failed: {e}")

    def _cleanup_ros_interface(self):
        """Clean up ROS interface, continuing on error."""
        if not self.interface:
            return
        try:
            self.interface.shutdown()
            self.interface = None
        except Exception as e:
            print(f"Warning: ROS interface shutdown failed: {e}")

    def _cleanup_ego_actor(self):
        """Destroy ego vehicle, continuing on error."""
        if not self.ego_actor:
            return
        try:
            self.ego_actor.destroy()
            self.ego_actor = None
        except Exception as e:
            print(f"Warning: Ego actor destruction failed: {e}")
