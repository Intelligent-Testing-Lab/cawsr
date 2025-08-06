import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import math
import random


class Waypoint(object):
    def __init__(self, x, y, z, node=None):
        self.x = x
        self.y = y
        self.z = z
        self.id = random.randint(0, 10000)

        self.node = node
        self.path_publisher = node.path_publisher

        self.client = CarlaDataProvider.get_client()

    def autoware_from_world_coords(self) -> Pose:
        """convert from carla world coordinates to autoware waypoints

        Returns:
            PoseStamped: position and time stamp
        """
        pose = Pose()

        ros_point = Point()
        ros_point.x = self.x
        ros_point.y = -self.y
        ros_point.z = self.z

        curr_location = carla.Location(self.x, self.y, self.z)
        orientation = (
            self.client.get_world()
            .get_map()
            .get_waypoint(
                curr_location, project_to_road=True, lane_type=carla.LaneType.Driving
            )
            .transform.rotation
        )

        qx, qy, qz, qw = quaternion_from_euler(
            math.radians(orientation.roll),
            -math.radians(orientation.pitch),
            -math.radians(round(orientation.yaw, 1)),
        )

        pose.position = ros_point
        pose.orientation = Quaternion(
            x=qx,
            y=qy,
            z=qz,
            w=qw,
        )

        self.pose = pose

        return pose

    def publish_route(self, poses: list[Pose]) -> None:
        path = Marker()
        path.header.frame_id = "map"
        path.header.stamp = self.node.get_clock().now().to_msg()

        path.poses = poses

        self.path_publisher.publish(path)

    def __str__(self) -> str:
        if self.pose:
            return f"{self.pose}"
        return f"x: {self.x}, y: {-self.y}, z: {self.z}"
