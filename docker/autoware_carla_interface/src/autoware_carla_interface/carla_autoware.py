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
# limitations under the License.sr/bin/env python

import signal
import time

import carla
import rclpy

import threading


from .carla_ros import carla_ros2_interface
from .modules.carla_data_provider import CarlaDataProvider
from .modules.carla_data_provider import GameTime
from .modules.carla_wrapper import SensorReceivedNoData
from .modules.carla_wrapper import SensorWrapper

from autoware_carla_interface_msgs.msg import EgoConfig, BridgeState


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
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()
            try:
                ego_action = self.sensor()
            except SensorReceivedNoData as e:
                raise RuntimeError(e)
            self.ego_actor.apply_control(ego_action)


class SensorListener(object):
    ego_config = "/bridge/ego_vehicle/config"  # publish sensor config type: EgoConfig
    bridge_state = "/bridge/state"  # once attached to the ego, publish ready message

    def __init__(self) -> None:
        self.sensor_config = None
        self.ego_name = None
        self.ego_model = None

        rclpy.init(args=None)
        self.node = rclpy.create_node("bridge_state")

        self._sensor_sub = self.node.create_subscription(
            EgoConfig, self.ego_config, self.sensor_listener_cb, 1
        )

        self._bridge_pub = self.node.create_publisher(BridgeState, self.bridge_state, 1)

        # spin the node on another thread
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)

        self.executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self.executor_thread.start()

    def _cleanup(self) -> None:
        self.node.destroy_node()
        self.executor_thread.join()

    def listen_for_message(self) -> dict:
        # wait for the sensor message is received
        while self.sensor_config is None and self.ego_name is None:
            time.sleep(1)
            self.node.get_logger().info("Listening for sensor message...")

        # got sensor message, format as needed and return as dict
        config = {}
        config["ego"] = {"ego_name": self.ego_name, "ego_model": self.ego_model}

        # convert the sensorconfig into a list of dict
        sensors = []
        for sensor in self.sensor_config:
            sensors.append({"type": sensor.sensor_type, "id": sensor.sensor_id})

        config["sensors"] = sensors

        return config

    def sensor_listener_cb(self, msg) -> None:
        self.node.get_logger().info("Received Sensor message")
        if msg is not None:  # sanity check
            self.node.get_logger().info("Valid message, updating state")
            self.sensor_config = msg.sensors
            self.ego_name = msg.ego_name
            self.ego_model = msg.ego_model

    def publish_bridge_state(self, state: bool) -> None:
        bridge_state = BridgeState()
        bridge_state.bridge_ready = state
        self.node.get_logger().info("Sent handshake: bridge is ready.")
        self._bridge_pub.publish(bridge_state)


class InitializeInterface(object):
    def __init__(self):
        # wait for ego to be spawned and receive sensor data
        self.sensor_listener = SensorListener()
        self.env_config = self.sensor_listener.listen_for_message()

        self.interface = carla_ros2_interface(self.env_config["sensors"])
        self.param_ = self.interface.get_param()
        self.world = None
        self.sensor_wrapper = None
        self.ego_actor = None
        self.prev_tick_wall_time = 0.0

        # Parameter for Initializing Carla World
        self.local_host = self.param_["host"]
        self.port = self.param_["port"]
        self.timeout = self.param_["timeout"]
        self.max_real_delta_seconds = 0.05
        self.sensor_listener.node.get_logger().info(
            "Loaded bridge and received sensor message"
        )

    def load_world(self):
        client = carla.Client(self.local_host, self.port)
        client.set_timeout(self.timeout)
        self.world = client.get_world()

        CarlaDataProvider.set_world(self.world)
        CarlaDataProvider.set_client(client)

        # here, add code to wait for ego to spawn and filter by ego name
        # for now, parameter passed in from ROS

        # find ego vehicle
        ego_rolename = self.env_config["ego"]["ego_name"]

        self.sensor_listener.node.get_logger().info("Searching for ego vehicle...")
        self._found_ego = False

        while not self._found_ego:
            CarlaDataProvider.get_world().tick()
            vehicles = CarlaDataProvider.get_world().get_actors().filter("vehicle.*")
            self.sensor_listener.node.get_logger().info(f"Vehicles found: {vehicles}")
            for vehicle in vehicles:
                if vehicle.attributes["role_name"] == ego_rolename:
                    self.ego_actor = vehicle
                    self.sensor_listener.node.get_logger().info(
                        f"Found ego {self.ego_actor.attributes['role_name']}"
                    )
                    self._found_ego = True
            time.sleep(1)

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

        # send ready message - potentially may need to be handshake message
        self.sensor_listener.publish_bridge_state(True)

        while self.bridge_loop.running:
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                delta_step = time.time() - self.prev_tick_wall_time
                if delta_step <= self.max_real_delta_seconds:
                    # Add a wait to match the max_real_delta_seconds
                    time.sleep(self.max_real_delta_seconds - delta_step)
                self.prev_tick_wall_time = time.time()
                self.bridge_loop._tick_sensor(timestamp)

    def _stop_loop(self, sign, frame):
        self.bridge_loop._stop_loop()

    def _cleanup(self):
        self.sensor_wrapper.cleanup()
        CarlaDataProvider.cleanup()
        if self.ego_actor:
            self.ego_actor.destroy()
            self.ego_actor = None

        if self.interface:
            self.interface.shutdown()
            self.interface = None


def main():
    carla_bridge = InitializeInterface()
    carla_bridge.load_world()
    signal.signal(signal.SIGINT, carla_bridge._stop_loop)
    carla_bridge.run_bridge()
    carla_bridge._cleanup()


if __name__ == "__main__":
    main()
