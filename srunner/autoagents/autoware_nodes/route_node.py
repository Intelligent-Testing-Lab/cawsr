from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

# Import the service types directly
from autoware_adapi_v1_msgs.srv import SetRoutePoints, ClearRoute

# You might explicitly import the Request types for clarity, though not strictly necessary
from autoware_adapi_v1_msgs.srv._set_route_points import SetRoutePoints_Request
from autoware_adapi_v1_msgs.srv._clear_route import ClearRoute_Request


class RouteNode(Node):
    last_goal = None
    last_waypoints = []

    set_route_points_service_name = "/api/routing/set_route_points"
    clear_route_service_name = "/api/routing/clear_route"
    change_route_points_service_name = "/api/routing/change_route_points"

    def __init__(self, autoware_state) -> None:
        # Initialize the Node base class with a unique name
        super().__init__("route_client_node")

        self.autoware_state = autoware_state

        # Create service clients, not publishers
        self.set_route_client = self.create_client(
            SetRoutePoints, self.set_route_points_service_name
        )
        self.clear_route_client = self.create_client(
            ClearRoute, self.clear_route_service_name
        )

        self.change_route_client = self.create_client(
            SetRoutePoints, self.change_route_points_service_name
        )

        # wait for the services to become available

        while not self.set_route_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for '{self.set_route_points_service_name}' service..."
            )
        self.get_logger().info(
            f"Service '{self.set_route_points_service_name}' available."
        )

        while not self.clear_route_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for '{self.clear_route_service_name}' service..."
            )
        self.get_logger().info(f"Service '{self.clear_route_service_name}' available.")

        while not self.change_route_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for '{self.change_route_points_service_name}' service..."
            )
        self.get_logger().info(
            f"Service '{self.change_route_points_service_name}' available."
        )

    def _assemble_route_msg(
        self, goal: Pose, waypoints: list[Pose]
    ) -> SetRoutePoints_Request:
        """Build a SetRoutePoints_Request message for use with
            - /api/routing/set_route_points
            - /api/routing/change_route_points

        Args:
            goal (Pose): Goal Pose
            waypoints (list[Pose]): List of waypoints

        Returns:
            SetRoutePoints_Request: Request Object
        """
        request = SetRoutePoints_Request()

        header_msg = Header()
        time_stamp = self.get_clock().now().to_msg()

        header_msg.frame_id = "map"
        header_msg.stamp = time_stamp

        request.header = header_msg
        request.goal = goal
        request.waypoints = waypoints

        return request

    def request_route(self, goal: Pose, waypoints: list[Pose]) -> None:
        """Send a request to the /api/routing/set_route_points service.

        Args:
            goal (Pose): The goal position.
            waypoints (list[Pose]): A list of waypoints to pass through.
        """
        self.get_logger().info("Sending route request...")

        request = self._assemble_route_msg(goal, waypoints)

        # Call the service asynchronously. This returns a Future object.
        future = self.set_route_client.call_async(request)

        # Add a callback to process the response when it arrives.
        future.add_done_callback(self.set_route_response_callback)

    def change_route(self, goal: Pose, waypoints: list[Pose]) -> None:
        """Send a request to the /api/routing/change_route_points service.

        Args:
            goal (Pose): The goal position.
            waypoints (list[Pose]): A list of waypoints to pass through.
        """
        self.get_logger().info("Sending route request...")

        request = self._assemble_route_msg(goal, waypoints)

        # Call the service asynchronously. This returns a Future object.
        future = self.change_route_client.call_async(request)

        # Add a callback to process the response when it arrives.
        future.add_done_callback(self.change_route_response_callback)

    def set_route_response_callback(self, future):
        """Callback to handle the response from the SetRoutePoints service."""
        try:
            # Get the result from the future object
            response = future.result()
            # Assuming Autoware services return a status field with success/message
            if response.status.success:
                self.autoware_state.sent_route = True
                self.get_logger().info("Route set successfully!")
            else:
                self.get_logger().warn(
                    f"Failed to set route: {response.status.message}"
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def request_clear_route(self):
        """Send a request to clear the route by calling /api/routing/clear_route service."""
        self.get_logger().info("Sending clear route request...")

        # Create an empty request object for the ClearRoute service
        request = ClearRoute_Request()  # Or ClearRoute()

        # Call the service asynchronously
        future = self.clear_route_client.call_async(request)
        future.add_done_callback(self.clear_route_response_callback)

    def change_route_response_callback(self, future):
        """Callback to handle the response from the ChangeRoute service."""
        try:
            response = future.result()
            if response.status.success:
                self.get_logger().info("Updated route successfully!")
            else:
                self.get_logger().warn(
                    f"Failed to update route, terminating scenario: {response.status.message}"
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def clear_route_response_callback(self, future):
        """Callback to handle the response from the ClearRoute service."""
        try:
            response = future.result()
            if response.status.success:
                self.get_logger().info("Route cleared successfully!")
            else:
                self.get_logger().warn(
                    f"Failed to clear route: {response.status.message}"
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
