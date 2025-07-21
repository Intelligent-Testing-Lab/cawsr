from typing import Optional

from rclpy.node import Node
from autoware_vehicle_msgs.msg import Engage
from autoware_adapi_v1_msgs.srv import InitializeLocalization
from geometry_msgs.msg import PoseWithCovarianceStamped

from srunner.autoagents.agent_state import autoware_state


class AutowareNode(Node):
    engage = "/autoware/engage"
    localize = "/api/localization/initialize"

    def __init__(self, autoware_state: autoware_state.AutowareState):
        self.engage_publisher = self.create_publisher(Engage, self.engage, 10)
        self.localize_publisher = self.create_publisher(
            InitializeLocalization, self.localize, 10
        )
        self.autoware_state = autoware_state

    def publish_engage(self, engage_state: bool) -> None:
        """publish inputted engage boolean to the /autoware/engage topic

        Args:
            engage_state (bool): engage value
        """
        engage_msg = Engage()
        engage_msg.engage = engage_state
        self.autoware_state.sent_engage = engage_state
        self.engage_publisher.publish(engage_msg)

    def publish_localize(
        self, global_pose: Optional[PoseWithCovarianceStamped]
    ) -> None:
        """publish localize message to /api/localization/initialize

        Args:
            global_pose (Optional[Pose]): rough guess of current position
        """
        if global_pose:
            self.localize_publisher.publish(global_pose)
        else:
            self.localize_publisher.publish()
