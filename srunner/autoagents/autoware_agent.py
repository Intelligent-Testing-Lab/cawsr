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

        self.autoware_state = autoware_state.AutowareState("ego_vehicle", None)

        self.carla_interface = InitializeInterface(self.config)

        self.state_node = state_node.StateNode(self.autoware_state)
        self.state_node.reset_autoware(self.config.town, self.config.ego_name)

        self.route_node = route_node.RouteNode(self.autoware_state)
        self.autoware_node = autoware_node.AutowareNode(self.autoware_state)

        self._multi_thread_executor = rclpy.executors.MultiThreadedExecutor()

        self._multi_thread_executor.add_node(self.route_node)
        self._multi_thread_executor.add_node(self.state_node)
        self._multi_thread_executor.add_node(self.autoware_node)
        self._multi_thread_executor.add_node(self.carla_interface.interface.ros2_node)  # type:ignore

        self._executor_thread = threading.Thread(
            target=self._multi_thread_executor.spin, daemon=True
        )
        self._executor_thread.start()

        self.sent_route = False
        self.initialised = False

        self.carla_interface.load_world()
        self.carla_interface.run_bridge()

    def set_route(self) -> None:
        self.agent_set_route = True

        self.goal_pose_world = self._global_plan_world_coord[-1]
        self.waypoints_world = self._global_plan_world_coord[:-1]

        # autoware cannot handle many waypoints, becomes unreliable
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
            node=self.autoware_node,
        )

    def destroy(self) -> None:
        """Cleanup"""
        logger.info("Sending shutdown signal to autoware...")
        self.state_node.reset_autoware(self.config.town, self.config.ego_name)
        logger.info("Waiting for shutdown. Starting Node cleanup")
        time.sleep(1)  # sleep for 1 second for sanity
        try:
            self.autoware_node.destroy_node()
            self.state_node.destroy_node()
            self.route_node.destroy_node()
            rclpy.shutdown()
            self._executor_thread.join()
        except RuntimeError:
            logger.info("Failed to clean up executor thread...")

    def cleanup(self) -> None:
        self.destroy()

    def run_step_init(self) -> bool:
        """Route Initialisation loop

        Ticks CARLA and Autoware, allowing the agent to localise and plan the route.
        Operates on a fixed tick budget to ensure determinism. If the agent goes over the budget, it is treated as a failure.

        """

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
        if self.counter % 20 == 0:
            logger.info(
                f"Ticked 1 second game-time, actual tick is {(time.perf_counter_ns() - self.last_tick) / 1e6}ms"
            )
            self.last_tick = time.perf_counter_ns()

        if not self.initialised:
            self.initialised = self.run_step_init()

            if self.initialised:
                logger.info("Set agent route!")

        # check if the current route is set
        if self.autoware_state.route_set() and not self.autoware_state.sent_engage:
            self.autoware_node.publish_engage(True)

        self.carla_interface.tick_bridge()
