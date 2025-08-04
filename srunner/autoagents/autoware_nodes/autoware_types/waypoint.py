import carla
from math import atan2
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class Waypoint(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        
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

        orientation = self._get_orientation()

        pose.position = ros_point
        pose.orientation = Quaternion(
            x=orientation["x"],
            y=orientation["y"],
            z=orientation["z"],
            w=orientation["w"],
        )

        return pose

    def _get_orientation(self) -> dict:
        curr_location = carla.Location(self.x, self.y, self.z)
        point1 = self.client.get_waypoint(
            curr_location, project_to_road=True, lane_type=carla.LaneType.Driving
        )
        
        # get next waypoint that is 0.5 meters away
        point2 = point1.next(0.5)
        
        dx = point2.transform.location.x - point1.transform.location.x
        dy = point2.transform.location.y - point1.transform.location.y
        yaw = atan2(dy, dx)
        
        # rounding is need by autoware to function properly
        qx, qy, qz, qw = quaternion_from_euler(0, 0, round(yaw, 1))
        
        return {
            "x": qx,
            "y": qy,
            "z": qz,
            "w": qw,
        }
        
