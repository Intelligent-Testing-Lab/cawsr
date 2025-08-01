from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class Waypoint(object):
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

        # rounding is need by autoware to function properly
        self.quaternion = quaternion_from_euler(0, 0, round(self.yaw, 1))

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

        pose.position = ros_point
        pose.orientation = Quaternion(
            w=self.quaternion[3],
            x=self.quaternion[0],
            y=self.quaternion[1],
            z=self.quaternion[2],
        )

        return pose
