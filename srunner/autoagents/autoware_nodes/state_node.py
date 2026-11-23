#!/bin/bash

# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from tier4_planning_msgs.msg import RouteState
from autoware_adapi_v1_msgs.msg import MotionState
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.msg import MrmState
from srunner.autoagents.agent_state import autoware_state
import logging

from autoware_cawsr_msgs.msg import AutowareRestart, AutowareShutdown

logger = logging.getLogger("scenario-runner")
logger.propagate = False


"""
uint16 state:
    NORMAL = 1
    MRM_OPERATING = 2
    MRM_SUCCEEDED = 3
    MRM_FAILED = 4

uint16 behaviour:
    NONE = 1
    EMERGENCY_STOP = 2
    COMFORTABLE_STOP = 3
    PULL_OVER = 4
"""
MRM_MAPPING = {
    "behaviour": {
        1: "NONE",
        2: "EMERGENCY_STOP",
        3: "COMFORTABLE_STOP",
        4: "PULL_OVER",
    },
    "state": {1: "NORMAL", 2: "MRM_OPERATING", 3: "MRM_SUCCEEDED", 4: "MRM_FAILED"},
}


class StateNode:
    route_state = "/planning/mission_planning/state"
    motion_state = "/api/motion/state"
    localize_state = "/api/localization/initialization_state"
    operation_state = "/api/operation_mode/state"
    mrm_state = "/api/fail_safe/mrm_state"

    reset_topic = "/autoware/restart"

    def __init__(self, autoware_state: autoware_state.AutowareState, node) -> None:
        self.autoware_state = autoware_state
        self.node = node

        self.route_subscriber = self.node.create_subscription(
            RouteState, self.route_state, self.route_state_cb, 10
        )
        self.motion_subscriber = self.node.create_subscription(
            MotionState, self.motion_state, self.motion_state_cb, 10
        )
        self.localize_state_subscriber = self.node.create_subscription(
            LocalizationInitializationState,
            self.localize_state,
            self.localize_state_cb,
            10,
        )

        self.operation_state_subscriber = self.node.create_subscription(
            OperationModeState, self.operation_state, self.operation_state_cb, 10
        )

        self.mrm_state_subscriber = self.node.create_subscription(
            MrmState, self.mrm_state, self.mrm_state_cb, 10
        )

        self.restart_autoware_pub = self.node.create_publisher(
            AutowareRestart, self.reset_topic, 10
        )

        self.shutdown_autoware_pub = self.node.create_publisher(
            AutowareShutdown, "/autoware/shutdown", 10
        )

    def route_state_cb(self, route_state_msg: RouteState) -> None:
        """Set the AutowareState attribute route_state

        Args:
            route_state_msg (RouteState): route state message received
        """
        self.autoware_state.route_state = route_state_msg.state
        logger.info(f"Route state: {self.autoware_state.route_state}")

    def motion_state_cb(self, motion_state_msg: MotionState) -> None:
        """set the AutowareState attribute motion_state

        Args:
            motion_state_msg (MotionState): motion state message received
        """

        self.autoware_state.motion_state = motion_state_msg.state
        logger.info(f"Motion state: {self.autoware_state.motion_state}")

    def localize_state_cb(
        self, localize_state_msg: LocalizationInitializationState
    ) -> None:
        """set the AutowareState attribute localize_state

        Args:
            localize_state_msg (LocalizationInitializationState): localization message received
        """
        self.autoware_state.localize_state = localize_state_msg.state
        logger.info(f"Localization state: {self.autoware_state.localize_state}")

    def operation_state_cb(
        self,
        operation_state_msg: OperationModeState,
    ) -> None:
        """Set the Operation mode state

        Args:
            operation_state_msg (OperationModeState): OperationMode state message from the AD-API
        """

        if self.autoware_state.operation_mode != operation_state_msg.mode:
            self.autoware_state.operation_mode = operation_state_msg.mode  # uint8
            logger.info(f"Operation Mode: {operation_state_msg.mode}")

        if (
            self.autoware_state.is_autoware_control
            != operation_state_msg.is_autoware_control_enabled
        ):
            self.autoware_state.is_autoware_control = (
                operation_state_msg.is_autoware_control_enabled
            )
            logger.info(
                f"Autoware Control: {operation_state_msg.is_autoware_control_enabled}"
            )

        if (
            self.autoware_state.is_autonomous_available
            != operation_state_msg.is_autonomous_available
        ):
            self.autoware_state.is_autonomous_available = (
                operation_state_msg.is_autonomous_available
            )
            logger.info(
                f"Autonomous Available: {operation_state_msg.is_autonomous_available}"
            )

        if self.autoware_state.is_in_transition != operation_state_msg.is_in_transition:
            self.autoware_state.is_in_transition = operation_state_msg.is_in_transition
            logger.info(f"Transition: {operation_state_msg.is_in_transition}")

    def mrm_state_cb(self, mrm_state_msg: MrmState) -> None:
        """Records the current MRM state and behaviour of Autoware

        uint16 state:
            NORMAL = 1
            MRM_OPERATING = 2
            MRM_SUCCEEDED = 3
            MRM_FAILED = 4

        uint16 behaviour:
            NONE = 1
            EMERGENCY_STOP = 2
            COMFORTABLE_STOP = 3
            PULL_OVER = 4

        Args:
            mrm_state_msg (MrmState): MRM state and behaviour message in autoware
        """

        if self.autoware_state.mrm_behaviour != mrm_state_msg.behaviour:
            self.autoware_state.mrm_behaviour = mrm_state_msg.behaviour
            logger.info(
                f"MRM Behaviour: {MRM_MAPPING['behaviour'][mrm_state_msg.behaviour]}"
            )

        if self.autoware_state.mrm_state != mrm_state_msg.state:
            self.autoware_state.mrm_state = mrm_state_msg.state
            logger.info(f"MRM State: {MRM_MAPPING['state'][mrm_state_msg.state]}")

    def reset_autoware(self, carla_map: str, ego_name: str):
        """Publishes an empty message to reset autoware."""

        msg = AutowareRestart()
        msg.carla_map = str(carla_map)
        msg.ego_name = str(ego_name)

        self.restart_autoware_pub.publish(msg)

    def shutdown_autoware(self):
        """Publishes an empty message to shutdown autoware."""

        # fill with dummy data, since this topic only handles shutting autoware down
        msg = AutowareShutdown()
        msg.carla_map = "Town01"
        msg.ego_name = "ego_vehicle"

        self.shutdown_autoware_pub.publish(msg)
