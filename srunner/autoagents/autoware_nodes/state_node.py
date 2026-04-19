#!/bin/bash

# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
from __future__ import annotations
from autoware_planning_msgs.msg import RouteState
from autoware_adapi_v1_msgs.msg import MotionState
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from srunner.autoagents.agent_state import autoware_state
from typing import TYPE_CHECKING
import logging

from autoware_cawsr_msgs.msg import AutowareRestart, AutowareShutdown, CawsrState

if TYPE_CHECKING:
    from srunner.scenariomanager.scenario_state import ScenarioStateType


logger = logging.getLogger("scenario-runner")
logger.propagate = False


class StateNode:
    route_state = "/planning/mission_planning/state"
    motion_state = "/api/motion/state"
    localize_state = "/api/localization/initialization_state"

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

        self.restart_autoware_pub = self.node.create_publisher(
            AutowareRestart, self.reset_topic, 10
        )

        self.shutdown_autoware_pub = self.node.create_publisher(
            AutowareShutdown, "/autoware/shutdown", 10
        )

        self.cawsr_state_publisher = self.node.create_publisher(
            CawsrState, "/cawsr/scenario_state", 10
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

    def publish_cawsr_state(self, current_state: ScenarioStateType) -> None:
        """Publish the current scenario state to a topic for CAWSR to consume

        Args:
            scenario_state (ScenarioStateType): the current scenario state, e.g. 0, 1, 2, 3
                uint8 NOT_STARTED = 0
                uint8 INITIALISING= 1
                uint8 RUNNING = 2
                uint8 FINISHED = 3
        """
        msg = CawsrState()
        msg.scenario_state = current_state.value
        logger.info(
            f"Publishing CAWSR state: {current_state.name} ({current_state.value})"
        )
        self.cawsr_state_publisher.publish(msg)

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
