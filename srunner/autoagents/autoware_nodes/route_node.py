#!/bin/bash

# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from rclpy.node import Node
from geometry_msgs.msg import Pose

# Import the service types directly
from autoware_adapi_v1_msgs.srv import SetRoutePoints, ClearRoute

# You might explicitly import the Request types for clarity, though not strictly necessary
from autoware_adapi_v1_msgs.srv._clear_route import ClearRoute_Request
from geometry_msgs.msg import PoseStamped

logger = logging.getLogger("scenario-runner")


class RouteNode():
    last_goal = None
    last_waypoints = []

    # These are service names, not topic names. Renamed for clarity.
    set_route_points_service_name = "/api/routing/set_route_points"
    clear_route_service_name = "/api/routing/clear_route"

    goal_topic = "/planning/mission_planning/goal"
    checkpoint_topic = "/planning/mission_planning/checkpoint"

    def __init__(self, autoware_state, node) -> None:
        # Initialize the Node base class with a unique name
        super().__init__("route_client_node")

        self.autoware_state = autoware_state
        self.node = node

        # Create service clients, not publishers
        self.set_route_client = self.node.create_client(
            SetRoutePoints, self.set_route_points_service_name
        )
        self.clear_route_client = self.node.create_client(
            ClearRoute, self.clear_route_service_name
        )

        self.goal_publisher = self.node.create_publisher(PoseStamped, self.goal_topic, 10)
        self.checkpoint_publisher = self.node.create_publisher(
            PoseStamped, self.checkpoint_topic, 10
        )

        # Good practice: Wait for the service server to be available before trying to call it
        logger.info(
            f"Waiting for '{self.set_route_points_service_name}' service..."
        )
        while not self.set_route_client.wait_for_service(timeout_sec=1.0):
            logger.info(
                f"Service '{self.set_route_points_service_name}' not available, waiting again..."
            )
        logger.info(
            f"Service '{self.set_route_points_service_name}' available."
        )

        logger.info(
            f"Waiting for '{self.clear_route_service_name}' service..."
        )
        while not self.clear_route_client.wait_for_service(timeout_sec=1.0):
            logger.info(
                f"Service '{self.clear_route_service_name}' not available, waiting again..."
            )
        logger.info(f"Service '{self.clear_route_service_name}' available.")

    def publish_route(self, goal: Pose, checkpoints: list[Pose]) -> None:
        """Responsible for publishing the end goal point and the obligatory checkpoints to visist

        Args:
            goal (Pose): goal position
            checkpoints (list[Pose]): checkpoints to visit before end goal
        """

        for checkpoint in checkpoints:
            self._publish_checkpoint(checkpoint)

        self._publish_goal(goal)

    def _publish_goal(self, goal_point: Pose) -> None:
        """publish the goal position

        Args:
            goal_point (Pose): end goal position
        """
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"

        goal.pose = goal_point
        self.goal_publisher.publish(goal)

    def _publish_checkpoint(self, checkpoint: Pose) -> None:
        """publish a single checkpoint position

        Args:
            checkpoint (Pose): checkpoint position
        """
        checkpoint_msg = PoseStamped()
        checkpoint_msg.header.stamp = self.get_clock().now().to_msg()
        checkpoint_msg.header.frame_id = "map"

        checkpoint_msg.pose = checkpoint
        self.checkpoint_publisher.publish(checkpoint_msg)

    def request_clear_route(self):
        """Send a request to clear the route by calling /api/routing/clear_route service."""
        logger.info("Sending clear route request...")

        # Create an empty request object for the ClearRoute service
        request = ClearRoute_Request()  # Or ClearRoute()

        # Call the service asynchronously
        future = self.clear_route_client.call_async(request)
        future.add_done_callback(self.clear_route_response_callback)

    def clear_route_response_callback(self, future):
        """Callback to handle the response from the ClearRoute service."""
        try:
            response = future.result()
            if response.status.success:
                logger.info("Route cleared successfully!")
            else:
                logger.warn(
                    f"Failed to clear route: {response.status.message}"
                )
        except Exception as e:
            logger.error(f"Service call failed: {e}")
