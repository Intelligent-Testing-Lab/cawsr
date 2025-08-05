import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import math


class Waypoint(object):
    def __init__(self, x, y, z, node=None):
        self.x = x
        self.y = y
        self.z = z

        self.node = node
        self.marker_pub = node.marker_publisher

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

    def _publish_marker(self, pose):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = self.node.get_clock().now().to_msg()

        marker.type = marker.ARROW
        marker.action = marker.ADD

        marker.pose = pose

        marker.scale.x = 0.5
        marker.scale.y = 0.05
        marker.scale.z = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def __str__(self) -> str:
        if self.pose:
            return f"{self.pose}"
        return f"x: {self.x}, y: {-self.y}, z: {self.z}"
