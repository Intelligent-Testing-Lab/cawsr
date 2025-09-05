from typing import Optional

from rclpy.node import Node
from autoware_vehicle_msgs.msg import Engage
from autoware_adapi_v1_msgs.srv import InitializeLocalization
from autoware_adapi_v1_msgs.srv._initialize_localization import (
    InitializeLocalization_Request,
)  # Explicitly import Request
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker


# Assuming this import path is correct for your project
from srunner.autoagents.agent_state import autoware_state


class AutowareNode(Node):
    engage_topic = "/autoware/engage"
    localize_service = "/api/localization/initialize"

    def __init__(self, autoware_state_instance: autoware_state.AutowareState):
        super().__init__("autoware_node")

        self.engage_publisher = self.create_publisher(Engage, self.engage_topic, 10)

        self.localize_client = self.create_client(
            InitializeLocalization, self.localize_service
        )

        self.marker_publisher = self.create_publisher(
            Marker, "visulaization_marker", 10
        )

        # Good practice: Wait for the service to be available
        self.get_logger().info(f"Waiting for '{self.localize_service}' service...")
        while not self.localize_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Service '{self.localize_service}' not available, waiting again..."
            )
        self.get_logger().info(f"Service '{self.localize_service}' available.")

        # Store the autoware_state instance
        self.autoware_state = autoware_state_instance

    def publish_engage(self, engage_state: bool) -> None:
        """Publish inputted engage boolean to the /autoware/engage topic

        Args:
            engage_state (bool): engage value
        """
        engage_msg = Engage()
        engage_msg.engage = engage_state
        self.autoware_state.sent_engage = (
            engage_state  # Assuming this updates your state
        )
        self.engage_publisher.publish(engage_msg)
        self.get_logger().info(f"Published engage state: {engage_state}")

    def request_localize(
        self,
        global_pose: Optional[
            PoseWithCovarianceStamped
        ] = None,  # Make it optional with default None
    ) -> None:
        """Send a request to the /api/localization/initialize service.

        Args:
            global_pose (Optional[PoseWithCovarianceStamped]): Rough guess of current position.
                                                               If None, an empty request is sent.
        """
        self.get_logger().info("Sending localization initialization request...")

        request = InitializeLocalization_Request()
        if global_pose:
            # Populate the request with the provided pose
            # Assuming the service definition has a field named 'pose' of type PoseWithCovarianceStamped
            request.pose = global_pose
        else:
            self.get_logger().info(
                "No initial global_pose provided, sending empty localization request."
            )

        # Call the service asynchronously
        future = self.localize_client.call_async(request)

        # Add a callback to process the response when it arrives
        future.add_done_callback(self.localize_response_callback)

    def localize_response_callback(self, future):
        """Callback to handle the response from the InitializeLocalization service."""
        try:
            response = future.result()
            # Assuming Autoware services return a common status message in the response
            if response.status.success:
                self.get_logger().info("Localization initialized successfully!")
            else:
                self.get_logger().warn(
                    f"Failed to initialize localization: {response.status.message}"
                )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
