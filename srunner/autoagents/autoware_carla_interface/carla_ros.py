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

from __future__ import annotations

import math
import queue

# pylint: disable=import-error
from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import GearReport
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import VelocityReport
from builtin_interfaces.msg import Time
import carla
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from tier4_vehicle_msgs.msg import ActuationStatusStamped
from transforms3d.euler import euler2quat

from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.autoagents.autoware_carla_interface.modules.carla_utils import (
    carla_location_to_ros_point,
)
from srunner.autoagents.autoware_carla_interface.modules.carla_utils import (
    carla_rotation_to_ros_quaternion,
)
from srunner.autoagents.autoware_carla_interface.modules.carla_utils import create_cloud
from srunner.autoagents.autoware_carla_interface.modules.carla_utils import (
    ros_pose_to_carla_transform,
)
from srunner.autoagents.autoware_carla_interface.modules.carla_wrapper import (
    SensorInterface,
)
from srunner.tools.CARLA_manager import CARLAManager
from srunner.scenarioconfigs.environment_configuration import EnvironmentConfig

import rclpy


class carla_ros2_interface(object):
    def __init__(self, node: rclpy.node.Node, config: EnvironmentConfig):
        self.sensor_interface = SensorInterface()
        self.config = config
        self.prev_timestamp = None
        self.prev_steer_output = 0.0
        self.tau = 0.2
        self.timestamp = 0.0
        self.ego_actor = None
        self.physics_control = None
        self.channels = 0
        self.id_to_sensor_type_map = {}
        self.id_to_camera_info_map = {}
        self.cv_bridge = CvBridge()
        self.first_ = True
        self._pending_initialpose = None
        self.pub_lidar = {}
        self.sensor_frequencies = {
            "top": 11,
            "left": 11,
            "right": 11,
            "camera": 11,
            "imu": 50,
            "status": 50,
            "pose": 20,
        }

        self.publish_prev_times = {
            sensor: -1000.0 for sensor in self.sensor_frequencies
        }

        self.delta = CARLAManager.FIXED_DELTA_SECONDS  # delta in seconds
        self.ros2_node = node

        self.clock_publisher = self.ros2_node.create_publisher(Clock, "/clock", 10)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=int(0))
        self.clock_publisher.publish(obj_clock)

        self.sensors = {
            "sensors": [sensor.sensor_dict() for sensor in self.config.sensor_config]
        }

        self.sub_control = self.ros2_node.create_subscription(
            ActuationCommandStamped,
            "/control/command/actuation_cmd",
            self.control_callback,
            1,
        )

        self.sub_vehicle_initialpose = self.ros2_node.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.initialpose_callback, 1
        )

        self._control_queue = queue.Queue(1)
        self._control_queue.put_nowait((0, carla.VehicleControl()))
        self._last_control = carla.VehicleControl()

        self.pub_pose_with_cov = self.ros2_node.create_publisher(
            PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", 1
        )
        self.pub_vel_state = self.ros2_node.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 1
        )
        self.pub_steering_state = self.ros2_node.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 1
        )
        self.pub_ctrl_mode = self.ros2_node.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 1
        )
        self.pub_gear_state = self.ros2_node.create_publisher(
            GearReport, "/vehicle/status/gear_status", 1
        )
        self.pub_actuation_status = self.ros2_node.create_publisher(
            ActuationStatusStamped, "/vehicle/status/actuation_status", 1
        )

        for sensor in self.sensors["sensors"]:
            self.id_to_sensor_type_map[sensor["id"]] = sensor["type"]
            if sensor["type"] == "sensor.camera.rgb":
                self.pub_camera = self.ros2_node.create_publisher(
                    Image, "/sensing/camera/traffic_light/image_raw", 1
                )
                self.pub_camera_info = self.ros2_node.create_publisher(
                    CameraInfo, "/sensing/camera/traffic_light/camera_info", 1
                )

                self.pub_camera_yolo_info = self.ros2_node.create_publisher(
                    CameraInfo, "/sensing/camera/camera4/camera_info", 1
                )

                self.pub_camera_yolo = self.ros2_node.create_publisher(
                    Image, "/sensing/camera/camera4/image_raw", 1
                )

            elif sensor["type"] == "sensor.lidar.ray_cast":
                if sensor["id"] in self.sensor_frequencies:
                    self.pub_lidar[sensor["id"]] = self.ros2_node.create_publisher(
                        PointCloud2,
                        f"/sensing/lidar/{sensor['id']}/pointcloud_before_sync",
                        10,  # lower qos depth as using best_reliability
                    )
                else:
                    self.ros2_node.get_logger().info(
                        "Please use Top, Right, or Left as the LIDAR ID"
                    )
            elif sensor["type"] == "sensor.other.imu":
                self.pub_imu = self.ros2_node.create_publisher(
                    Imu, "/sensing/imu/tamagawa/imu_raw", 1
                )
            else:
                self.ros2_node.get_logger().info(
                    f"No Publisher for {sensor['type']} Sensor"
                )
                pass

    def __call__(self, timestamp=None):
        if timestamp is None:
            timestamp = (
                CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
            )
        elif hasattr(timestamp, "elapsed_seconds"):
            timestamp = timestamp.elapsed_seconds

        input_data = self.sensor_interface.get_data(expected_time=timestamp)
        control = self.run_step(input_data, timestamp)
        return control

    def checkFrequency(self, sensor):
        time_delta = GameTime.get_time() - self.publish_prev_times[sensor]
        if time_delta <= 0.0:
            return False
        if 1.0 / time_delta >= self.sensor_frequencies[sensor]:
            return True
        return False

    def get_msg_header(self, frame_id, timestamp=None):
        """Obtain and modify ROS message header."""
        header = Header()
        header.frame_id = frame_id
        ts = timestamp if timestamp is not None else self.timestamp
        seconds = int(ts)
        nanoseconds = int((ts - int(ts)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header

    def lidar(self, carla_lidar_measurement, id_, timestamp=None):
        """Transform the received lidar measurement into a ROS point cloud message."""
        if id_ not in self.publish_prev_times or id_ not in self.pub_lidar:
            return
        if self.checkFrequency(id_):
            return
        self.publish_prev_times[id_] = GameTime.get_time()

        header = self.get_msg_header(
            frame_id="velodyne_top_changed", timestamp=timestamp
        )

        lidar_data = numpy.frombuffer(
            carla_lidar_measurement.raw_data, dtype=numpy.float32
        ).reshape(-1, 4)

        point_count = lidar_data.shape[0]
        buf = numpy.empty(point_count, dtype=[
            ("x", "f4"), ("y", "f4"), ("z", "f4"),
            ("intensity", "u1"), ("return_type", "u1"), ("channel", "u2"),
        ])

        buf["x"] = lidar_data[:, 0]
        buf["y"] = -lidar_data[:, 1]
        buf["z"] = lidar_data[:, 2]
        buf["intensity"] = numpy.clip(lidar_data[:, 3], 0, 1) * 255
        buf["return_type"] = 0

        channels = self.sensors["sensors"][1]["channels"]
        ch_starts = [0]
        for i in range(channels):
            ch_starts.append(ch_starts[-1] + carla_lidar_measurement.get_point_count(i))
        for i in range(channels):
            buf["channel"][ch_starts[i] : ch_starts[i + 1]] = i

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
            PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
            PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
        ]

        point_cloud_msg = create_cloud(header, fields, buf)
        self.pub_lidar[id_].publish(point_cloud_msg)

    def initialpose_callback(self, data):
        """Transform RVIZ initial pose to CARLA."""
        pose = data.pose.pose
        pose.position.z += 2.0
        carla_pose_transform = ros_pose_to_carla_transform(pose)
        self._pending_initialpose = carla_pose_transform

    def pose(self, timestamp=None):
        """Transform odometry data to Pose and publish Pose with Covariance message."""
        if self.checkFrequency("pose"):
            return
        self.publish_prev_times["pose"] = GameTime.get_time()

        header = self.get_msg_header(frame_id="map", timestamp=timestamp)
        out_pose_with_cov = PoseWithCovarianceStamped()
        pose_carla = Pose()
        pose_carla.position = carla_location_to_ros_point(
            self.ego_actor.get_transform().location
        )  # type: ignore
        pose_carla.orientation = carla_rotation_to_ros_quaternion(
            self.ego_actor.get_transform().rotation  # type: ignore
        )
        out_pose_with_cov.header = header
        out_pose_with_cov.pose.pose = pose_carla
        out_pose_with_cov.pose.covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.pub_pose_with_cov.publish(out_pose_with_cov)

    def _build_camera_info(self, camera_actor):
        """Build camera info."""
        camera_info = CameraInfo()
        camera_info.width = camera_actor.width
        camera_info.height = camera_actor.height
        camera_info.distortion_model = "plumb_bob"
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (2.0 * math.tan(camera_actor.fov * math.pi / 360.0))
        fy = fx
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._camera_info = camera_info

    def camera(self, carla_camera_data, timestamp=None):
        """Transform the received carla camera data into a ROS image and info message and publish."""
        while self.first_:
            self._camera_info_ = self._build_camera_info(carla_camera_data)
            self.first_ = False

        if self.checkFrequency("camera"):
            return
        self.publish_prev_times["camera"] = GameTime.get_time()

        image_data_array = numpy.ndarray(
            shape=(carla_camera_data.height, carla_camera_data.width, 4),
            dtype=numpy.uint8,
            buffer=carla_camera_data.raw_data,
        )
        # cspell:ignore interp bgra
        img_msg = self.cv_bridge.cv2_to_imgmsg(image_data_array, encoding="bgra8")
        img_msg.header = self.get_msg_header(
            frame_id="traffic_light_left_camera/camera_optical_link",
            timestamp=timestamp,
        )
        cam_info = self._camera_info
        cam_info.header = img_msg.header
        self.pub_camera_info.publish(cam_info)
        self.pub_camera.publish(img_msg)

        # build another message to publish for yolo
        img_msg.header = self.get_msg_header(
            frame_id="camera4/camera_optical_link", timestamp=timestamp
        )

        cam_info = self._camera_info
        cam_info.header = img_msg.header
        self.pub_camera_yolo_info.publish(cam_info)
        self.pub_camera_yolo.publish(img_msg)

    def imu(self, carla_imu_measurement, timestamp=None):
        """Transform a received imu measurement into a ROS Imu message and publish Imu message."""
        if self.checkFrequency("imu"):
            return
        self.publish_prev_times["imu"] = GameTime.get_time()

        imu_msg = Imu()
        imu_msg.header = self.get_msg_header(
            frame_id="tamagawa/imu_link_changed", timestamp=timestamp
        )
        imu_msg.angular_velocity.x = -carla_imu_measurement.gyroscope.x
        imu_msg.angular_velocity.y = carla_imu_measurement.gyroscope.y
        imu_msg.angular_velocity.z = -carla_imu_measurement.gyroscope.z

        imu_msg.linear_acceleration.x = carla_imu_measurement.accelerometer.x
        imu_msg.linear_acceleration.y = -carla_imu_measurement.accelerometer.y
        imu_msg.linear_acceleration.z = carla_imu_measurement.accelerometer.z

        roll = math.radians(carla_imu_measurement.transform.rotation.roll)
        pitch = -math.radians(carla_imu_measurement.transform.rotation.pitch)
        yaw = -math.radians(carla_imu_measurement.transform.rotation.yaw)

        quat = euler2quat(roll, pitch, yaw)
        imu_msg.orientation.w = quat[0]
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]

        self.pub_imu.publish(imu_msg)

    def first_order_steering(self, steer_input):
        """First order steering model."""
        timestamp = self.timestamp
        if self.prev_timestamp is None:
            self.prev_timestamp = timestamp

        dt = timestamp - self.prev_timestamp  # type: ignore
        if dt <= 0.0:
            return self.prev_steer_output

        steer_output = self.prev_steer_output + (
            steer_input - self.prev_steer_output
        ) * (dt / (self.tau + dt))
        self.prev_steer_output = steer_output
        self.prev_timestamp = timestamp
        return steer_output

    def control_callback(self, in_cmd):
        """Convert and publish CARLA Ego Vehicle Control to AUTOWARE."""
        out_cmd = carla.VehicleControl()
        out_cmd.throttle = in_cmd.actuation.accel_cmd
        # convert base on steer curve of the vehicle
        steer_curve = self.physics_control.steering_curve  # type: ignore
        current_vel = self.ego_actor.get_velocity()  # type: ignore
        max_steer_ratio = numpy.interp(
            abs(current_vel.x), [v.x for v in steer_curve], [v.y for v in steer_curve]
        )
        out_cmd.steer = (
            self.first_order_steering(-in_cmd.actuation.steer_cmd) * max_steer_ratio
        )
        out_cmd.brake = in_cmd.actuation.brake_cmd

        try:
            self._control_queue.put_nowait((self.timestamp, out_cmd))
            self._last_control = out_cmd
        except queue.Full:
            pass

    def ego_status(self):
        """Publish ego vehicle status."""
        if self.checkFrequency("status"):
            return

        self.publish_prev_times["status"] = GameTime.get_time()

        # convert velocity from cartesian to ego frame
        trans_mat = numpy.array(self.ego_actor.get_transform().get_matrix()).reshape(
            4, 4
        )  # type: ignore
        rot_mat = trans_mat[0:3, 0:3]
        inv_rot_mat = rot_mat.T
        vel_vec = numpy.array(
            [
                self.ego_actor.get_velocity().x,  # type: ignore
                self.ego_actor.get_velocity().y,  # type: ignore
                self.ego_actor.get_velocity().z,  # type: ignore
            ]
        ).reshape(3, 1)
        ego_velocity = (inv_rot_mat @ vel_vec).T[0]

        out_vel_state = VelocityReport()
        out_steering_state = SteeringReport()
        out_ctrl_mode = ControlModeReport()
        out_gear_state = GearReport()
        out_actuation_status = ActuationStatusStamped()

        out_vel_state.header = self.get_msg_header(frame_id="base_link")
        out_vel_state.longitudinal_velocity = ego_velocity[0]
        out_vel_state.lateral_velocity = ego_velocity[1]
        out_vel_state.heading_rate = (
            self.ego_actor.get_transform()
            .transform_vector(self.ego_actor.get_angular_velocity())
            .z  # type: ignore
        )

        out_steering_state.stamp = out_vel_state.header.stamp
        out_steering_state.steering_tire_angle = -math.radians(
            self.ego_actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)  # type: ignore
        )

        out_gear_state.stamp = out_vel_state.header.stamp
        out_gear_state.report = GearReport.DRIVE

        out_ctrl_mode.stamp = out_vel_state.header.stamp
        out_ctrl_mode.mode = ControlModeReport.AUTONOMOUS

        control = self.ego_actor.get_control()  # type: ignore
        out_actuation_status.header = self.get_msg_header(frame_id="base_link")
        out_actuation_status.status.accel_status = control.throttle
        out_actuation_status.status.brake_status = control.brake
        out_actuation_status.status.steer_status = -control.steer

        self.pub_actuation_status.publish(out_actuation_status)
        self.pub_vel_state.publish(out_vel_state)
        self.pub_steering_state.publish(out_steering_state)
        self.pub_ctrl_mode.publish(out_ctrl_mode)
        self.pub_gear_state.publish(out_gear_state)

    def run_step(self, input_data, timestamp):
        self.timestamp = timestamp

        if self._pending_initialpose is not None:
            if self.ego_actor is not None:
                self.ego_actor.set_transform(self._pending_initialpose)
            else:
                print("Can't find Ego Vehicle")
            self._pending_initialpose = None

        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=seconds, nanosec=nanoseconds)
        self.clock_publisher.publish(obj_clock)

        # publish data of all sensors
        for key, data in input_data.items():
            sensor_type = self.id_to_sensor_type_map[key]
            sensor_timestamp = data[0]
            sensor_data = data[1]
            if sensor_type == "sensor.camera.rgb":
                self.camera(sensor_data, timestamp=sensor_timestamp)
            elif sensor_type == "sensor.other.gnss":
                self.pose(timestamp=sensor_timestamp)
            elif sensor_type == "sensor.lidar.ray_cast":
                self.lidar(sensor_data, key, timestamp=sensor_timestamp)
            elif sensor_type == "sensor.other.imu":
                self.imu(sensor_data, timestamp=sensor_timestamp)
            else:
                self.ros2_node.get_logger().info(f"No Publisher for [{key}] Sensor")

        self.ego_status()

        try:
            _, control = self._control_queue.get_nowait()
            self._last_control = control
        except queue.Empty:
            control = self._last_control

        return control

    def shutdown(self):
        self.ros2_node.destroy_node()
