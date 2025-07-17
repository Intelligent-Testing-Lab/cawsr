from rclpy.node import Node
from tier4_planning_msgs.msg import RouteState
from autoware_adapi_v1_msgs.msg import MotionState
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from ___ import AutowareState


class StateNode(Node):
    autoware_state = AutowareState()

    route_state = "/planning/mission_planning/state"
    motion_state = "/api/motion/state"
    localize_state = "/api/localization/initialization_state"

    def __init__(self) -> None:
        self.route_subscriber = self.create_subscriber(
            RouteState, self.route_state, self.route_state_cb, 10
        )
        self.motion_subscriber = self.create_subscriber(
            MotionState, self.motion_state, self.motion_state_cb, 10
        )
        self.localize_state_subscriber = self.create_subscriber(
            LocalizationInitializationState,
            self.localize_state,
            self.localize_state_cb,
            10,
        )

    def route_state_cb(self, route_state_msg: RouteState) -> None:
        """Set the AutowareState attribute route_state

        Args:
            route_state_msg (RouteState): route state message received
        """
        self.autoware_state.route_state = route_state_msg.state

    def motion_state_cb(self, motion_state_msg: MotionState) -> None:
        """set the AutowareState attribute motion_state

        Args:
            motion_state_msg (MotionState): motion state message received
        """
        self.autoware_state.motion_state = motion_state_msg.state

    def localize_state_cb(
        self, localize_state_msg: LocalizationInitializationState
    ) -> None:
        """set the AutowareState attribute localize_state

        Args:
            localize_state_msg (LocalizationInitializationState): localization message received
        """
        self.autoware_state.localize_state = localize_state_msg.state
