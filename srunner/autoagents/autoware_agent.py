#!/bin/bash

# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from srunner.autoagents.autonomous_agent import AutonomousAgent

from srunner.autoagents.autoware_nodes.autoware_types import waypoint
from srunner.autoagents.autoware_nodes import autoware_node
from srunner.autoagents.autoware_nodes import route_node
from srunner.autoagents.autoware_nodes import state_node

from srunner.autoagents.agent_state import autoware_state
from srunner.scenarioconfigs.environment_configuration import EnvironmentConfig
from srunner.autoagents.autoware_carla_interface.carla_autoware import (
    InitializeInterface,
)
from srunner.scenariomanager.timer import GameTime
from srunner.tools.CARLA_manager import CARLAManager

import threading
import rclpy
import time
import logging


logger = logging.getLogger("scenario-runner")
logger.propagate = False


class AutowareAgent(AutonomousAgent):
    timestamp = None
    agent_set_route = False
    counter = 0
    last_tick = time.perf_counter_ns()

    def setup(self, config: EnvironmentConfig) -> None:
        """Setup the Autoware Agent.
            - Initialise the state
            - Setup nodes

        Args:
            config (EnvironmentConfig | None): environment configuration file
        """

        rclpy.init(args=None)

        self.config = config
        self.tick_delta = CARLAManager.FIXED_DELTA_SECONDS

        # initialise autoware state object
        self.autoware_state = autoware_state.AutowareState("ego_vehicle", None)

        self._node = rclpy.create_node("cawsr_bridge")
        self._node_state = rclpy.create_node("autoware_state_node")

        self.carla_interface = InitializeInterface(self.config, self._node)

        self.state_node = state_node.StateNode(self.autoware_state, self._node_state)
        self.state_node.reset_autoware(self.config.town, self.config.ego_name)

        self.route_node = route_node.RouteNode(self.autoware_state, self._node_state)
        self.autoware_node = autoware_node.AutowareNode(
            self.autoware_state, self._node_state
        )

        try:
            # run state node and cawsr bridge in separate executors
            self._executors = [
                rclpy.executors.SingleThreadedExecutor(),
                rclpy.executors.SingleThreadedExecutor(),
            ]

            self._executors[0].add_node(self._node)
            self._executors[1].add_node(self._node_state)

            self._executor_threads = [
                threading.Thread(target=self._executors[0].spin, daemon=True),
                threading.Thread(target=self._executors[1].spin, daemon=True),
            ]
        except rclpy.executors.ExternalShutdownException:
            logger.info("Node Executor shutdown externally...")

        self.sent_route = False

        self.carla_interface.load_world()
        self.carla_interface.run_bridge()

        for thread in self._executor_threads:
            thread.start()

    def set_route(self) -> None:
        self.agent_set_route = True

        self.goal_pose_world = self._global_plan_world_coord[-1]
        self.waypoints_world = self._global_plan_world_coord[:-1]

        n_waypoints = len(self.waypoints_world)
        segment_size = int(n_waypoints / 3)

        if segment_size > 1:
            self.waypoints_world = self.waypoints_world[0::segment_size]

        logger.info("Clearing route...")
        self.route_node.request_clear_route()

    def _convert_to_waypoint(self, point):
        """Returns a waypoint

        Args:
            point (Point): Point to convert
        """
        return waypoint.Waypoint(
            point[0].location.x,
            point[0].location.y,
            point[0].location.z,
            node=self._node,
        )

    def cleanup(self) -> None:
        self.destroy(cleanup=True)

    def destroy(self, cleanup=False) -> None:
        """Cleanup"""
        logger.info("Sending shutdown signal to autoware...")
        if not cleanup:
            self.state_node.reset_autoware(self.config.town, self.config.ego_name)
        else:
            self.state_node.shutdown_autoware()

        logger.info("Waiting for shutdown. Starting Node cleanup")
        time.sleep(1)  # sleep for 1 second for sanity
        try:
            self._node.destroy_node()
            self._node_state.destroy_node()
            rclpy.shutdown()
            for thread in self._executor_threads:
                thread.join()
        except RuntimeError:
            logger.info("Failed to clean up executor thread...")

    def run_step_init(self) -> bool:
        """Route Initialisation loop

        Ticks CARLA and Autoware, allowing the agent to localise and plan the route.
        Operates on a fixed tick budget to ensure determinism. If the agent goes over the budget, it is treated as a failure.
        """

        self.carla_interface.tick_bridge(self._carla_timestamp)

        if not self.agent_set_route:
            self.set_route()

        if (
            self.autoware_state.is_ready_publish_route()
            and self.agent_set_route
            and not self.sent_route
        ):
            waypoints = []
            goal_pose = self._convert_to_waypoint(
                self.goal_pose_world
            ).autoware_from_world_coords()

            for waypoint in self.waypoints_world:
                waypoints.append(
                    self._convert_to_waypoint(waypoint).autoware_from_world_coords()
                )

            self.route_node.publish_route(goal_pose, waypoints)
            self.sent_route = True

        # check if the current route is set and we are able to send engage
        if self.autoware_state.route_set() and not self.autoware_state.sent_engage:
            return True

        return False

    def run_step(self) -> None:
        """Tick method containing all logic based on autoware state"""
        self.counter += 1
        if self.counter % int(1 / self.tick_delta) == 0:
            logger.info(
                f"Ticked 1 second game-time, Current time: {GameTime.get_time():.2f}s, Current tick: {self.counter}. Ratio of 1s:{time.perf_counter_ns() - self.last_tick}s (Sim vs Wall)"
            )

            self.last_tick = time.perf_counter_ns()

        # check if the current route is set and we can publish engage
        if self.autoware_state.route_set() and not self.autoware_state.sent_engage:
            self.autoware_node.publish_engage(True)

        self.carla_interface.tick_bridge(self._carla_timestamp)
