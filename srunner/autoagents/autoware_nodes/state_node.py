from rclpy.node import Node
from tier4_planning_msgs.msg import RouteState
from autoware_adapi_v1_msgs.msg import MotionState
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from srunner.autoagents.agent_state import autoware_state
from autoware_carla_interface.msg import EgoConfig, BridgeState


class StateNode(Node):
    route_state = "/planning/mission_planning/state"
    motion_state = "/api/motion/state"
    localize_state = "/api/localization/initialization_state"

    ego_config = "/bridge/ego_vehicle/config"  # publish sensor config type: EgoConfig
    bridge_state = "/bridge/state"

    def __init__(self, autoware_state: autoware_state.AutowareState) -> None:
        super().__init__("state_node")
        self.autoware_state = autoware_state

        self.route_subscriber = self.create_subscription(
            RouteState, self.route_state, self.route_state_cb, 10
        )
        self.motion_subscriber = self.create_subscription(
            MotionState, self.motion_state, self.motion_state_cb, 10
        )
        self.localize_state_subscriber = self.create_subscription(
            LocalizationInitializationState,
            self.localize_state,
            self.localize_state_cb,
            10,
        )
        self.autoware_state_subscriber = self.create_subscription(
            BridgeState, self.bridge_state, self.bridge_state_cb, 10
        )

        self.ego_config_publisher = self.create_publisher(
            EgoConfig, self.ego_config, 10
        )

    def bridge_state_cb(self, bridge_state_msg: BridgeState):
        """Set the AutowareState attribute bridge_state

        Args:
            bridge_state_msg (BridgeState): bridge state boolean value
        """
        self.bridge_state = bridge_state_msg.bridge_ready

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
