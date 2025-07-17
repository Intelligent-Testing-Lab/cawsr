from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped


class Waypoint(object):
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

        # rounding is need by autoware to function properly
        self.quaternion = quaternion_from_euler(0, 0, round(self.yaw, 1))

    def autoware_from_world_coords(self) -> PoseStamped:
        """convert from carla world coordinates to autoware waypoints

        Returns:
            PoseStamped: position and time stamp
        """
        pose = PoseStamped()

        pose.header.frame_id = "map"
        return
