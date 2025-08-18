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

from __future__ import print_function

import logging
from queue import Empty
from queue import Queue

import carla
import numpy as np

from .carla_data_provider import CarlaDataProvider


# Sensor Wrapper for Agent
class SensorReceivedNoData(Exception):
    """Exceptions when no data received from the sensors."""


class GenericMeasurement(object):
    def __init__(self, data, frame):
        self.data = data
        self.frame = frame


class CallBack(object):
    def __init__(self, tag, sensor, data_provider):
        self._tag = tag
        self._data_provider = data_provider

        self._data_provider.register_sensor(tag, sensor)

    def __call__(self, data):
        if isinstance(data, carla.Image):
            self._parse_image_cb(data, self._tag)
        elif isinstance(data, carla.LidarMeasurement):
            self._parse_lidar_cb(data, self._tag)
        elif isinstance(data, carla.GnssMeasurement):
            self._parse_gnss_cb(data, self._tag)
        elif isinstance(data, carla.IMUMeasurement):
            self._parse_imu_cb(data, self._tag)
        elif isinstance(data, GenericMeasurement):
            self._parse_pseudo_sensor(data, self._tag)
        else:
            logging.error("No callback method for this sensor.")

    # Parsing CARLA physical Sensors
    def _parse_image_cb(self, image, tag):
        self._data_provider.update_sensor(tag, image, image.frame)

    def _parse_lidar_cb(self, lidar_data, tag):
        self._data_provider.update_sensor(tag, lidar_data, lidar_data.frame)

    def _parse_imu_cb(self, imu_data, tag):
        self._data_provider.update_sensor(tag, imu_data, imu_data.frame)

    def _parse_gnss_cb(self, gnss_data, tag):
        array = np.array(
            [gnss_data.latitude, gnss_data.longitude, gnss_data.altitude],
            dtype=np.float64,
        )
        self._data_provider.update_sensor(tag, array, gnss_data.frame)

    def _parse_pseudo_sensor(self, package, tag):
        self._data_provider.update_sensor(tag, package.data, package.frame)


class SensorInterface(object):
    def __init__(self):
        self._sensors_objects = {}
        self._new_data_buffers = Queue()
        self._queue_timeout = 10
        self.tag = ""

    def register_sensor(self, tag, sensor):
        self.tag = tag
        if tag in self._sensors_objects:
            raise ValueError(f"Duplicated sensor tag [{tag}]")

        self._sensors_objects[tag] = sensor

    def update_sensor(self, tag, data, timestamp):
        if tag not in self._sensors_objects:
            raise ValueError(f"Sensor with tag [{tag}] has not been created")

        self._new_data_buffers.put((tag, timestamp, data))

    def get_data(self):
        try:
            data_dict = {}
            while len(data_dict.keys()) < len(self._sensors_objects.keys()):
                sensor_data = self._new_data_buffers.get(True, self._queue_timeout)
                data_dict[sensor_data[0]] = (sensor_data[1], sensor_data[2])
        except Empty:
            raise SensorReceivedNoData(
                f"Sensor with tag [{self.tag}] took too long to send its data"
            )

        return data_dict


# Sensor Wrapper


class SensorWrapper(object):
    _agent = None
    _sensors_list = []

    def __init__(self, agent):
        self._agent = agent

    def __call__(self):
        return self._agent()

    def setup_sensors(self, vehicle, debug_mode=False):
        """Find all sensors that belong to the ego_vehicle parameter"""

        # instead of creating a sensor
        # use the ego name to find them (filter)
        # takes in parameter of sensor ID and type
        sensor_types = [sensor["type"] for sensor in self._agent.sensors]
        sensor_ids = [sensor["id"] for sensor in self._agent.sensors]
        type_to_id_map = dict(zip(sensor_types, sensor_ids))

        # get all world sensors
        for sensor in CarlaDataProvider.get_world().get_actors().filter("sensor.*"):
            attatched_to_ego = (
                sensor.parent.attributes["role_name"] == vehicle.attributes["role_name"]
            )
            if attatched_to_ego:
                # setup callback - sensor belongs to ego and was spawned by Scenario runner
                self._agent.ros2_node.get_logger().info(
                    f"Setup sensor of id {type_to_id_map[sensor.attributes['ros_name']]} and type {sensor.attributes['ros_name']}"
                )
                self._agent.ros2_node.get_logger().info(
                    f"Sensor Attributes {str(sensor.attributes)}"
                )
                sensor.listen(
                    CallBack(
                        type_to_id_map[sensor.attributes["ros_name"]],
                        sensor,
                        self._agent.sensor_interface,
                    )
                )
                self._sensors_list.append(sensor)

    def cleanup(self):
        """Cleanup sensors."""
        for i, _ in enumerate(self._sensors_list):
            if self._sensors_list[i] is not None:
                self._sensors_list[i].stop()
                self._sensors_list[i].destroy()
                self._sensors_list[i] = None
        self._sensors_list = []
