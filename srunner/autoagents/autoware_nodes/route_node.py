from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from autoware_adapi_v1_msgs.srv import SetRoutePoints, ClearRoute


class RouteNode(Node):
    last_goal = None
    last_waypoints = []

    set_route_points = "/api/routing/set_route_points"
    clear_route = "/api/routing/clear_route"

    def __init__(self) -> None:
        self.route_publisher = self.create_publisher(
            SetRoutePoints, self.set_route_points, 10
        )
        self.clear_publish = self.create_publisher(ClearRoute, self.clear_route, 10)

    def publish_route(self, goal: Pose, waypoints: list[Pose]) -> None:
        """Publish route to /api/routing/set_route_points

        Args:
            goal (Pose): goal position
            waypoints (list[Pose]): list of waypoints to pass through
        """
        header_msg = Header()
        route_points_msg = SetRoutePoints()

        time_stamp = self.get_clock().now().to_msg()

        header_msg.frame_id = "map"
        header_msg.frame_id = time_stamp

        route_points_msg.header = header_msg
        route_points_msg.pose = goal
        route_points_msg.wayponts = waypoints

        self.route_publisher.publish(route_points_msg)

    def publish_clear(self):
        """clear route by publishing to /api/routing/clear_route"""
        self.clear_publish.publish()
