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
import math


logger = logging.getLogger("scenario-runner")
logger.propagate = False


def _silent_spin(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass


class AutowareAgent(AutonomousAgent):
    MAX_ROUTE_RETRIES = 10

    timestamp = None
    agent_set_route = False
    scenario_loaded = False
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

        try:
            # run state node and cawsr bridge in separate executors
            self._executors = [
                rclpy.executors.SingleThreadedExecutor(),
                rclpy.executors.SingleThreadedExecutor(),
            ]

            self._executors[0].add_node(self._node)
            self._executors[1].add_node(self._node_state)

            self._executor_threads = [
                threading.Thread(
                    target=_silent_spin, args=(self._executors[0],), daemon=True
                ),
                threading.Thread(
                    target=_silent_spin, args=(self._executors[1],), daemon=True
                ),
            ]
        except rclpy.executors.ExternalShutdownException:
            logger.info("Node Executor shutdown externally...")

        for thread in self._executor_threads:
            thread.start()

        self.state_node.reset_autoware(self.config.town, self.config.ego_name)

        self.route_node = route_node.RouteNode(self.autoware_state, self._node_state)
        self.autoware_node = autoware_node.AutowareNode(
            self.autoware_state, self._node_state
        )

        self._reset_route_state()
        self._route_retry_count = 0

        self.carla_interface.load_world()
        self.carla_interface.run_bridge()

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

    def _reset_route_state(self):
        self.sent_route = False
        self._route_was_calculating = False

    def _retry_route(self):
        if self._route_retry_count < self.MAX_ROUTE_RETRIES:
            self._route_retry_count += 1
            self._reset_route_state()
            logger.info("Clearing route before retry...")
            self.route_node.request_clear_route()
        else:
            logger.error(
                f"Route setting failed after {self.MAX_ROUTE_RETRIES} attempts."
            )
            self.autoware_state.route_failed_permanently = True

    @property
    def route_failed_permanently(self):
        return self.autoware_state.route_failed_permanently

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

    def run_step(self) -> None:
        """Tick method containing all logic based on autoware state"""
        self.counter += 1
        if self.counter % int(1 / self.tick_delta) == 0:
            logger.info(
                f"Ticked 1 second game-time, Current time: {GameTime.get_time():.2f}s, Current tick: {self.counter}. Ratio of 1s:{time.perf_counter_ns() - self.last_tick}s (Sim vs Wall)"
            )

            self.last_tick = time.perf_counter_ns()

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

            if len(self._global_plan_world_coord) >= 2:
                prev = self._global_plan_world_coord[-2]
                curr = self.goal_pose_world
                dx = curr[0].location.x - prev[0].location.x
                dy = curr[0].location.y - prev[0].location.y
                approach_yaw = math.atan2(dy, dx)
                half_yaw = -approach_yaw / 2.0
                goal_pose.orientation.z = math.sin(half_yaw)
                goal_pose.orientation.w = math.cos(half_yaw)
                goal_pose.orientation.x = 0.0
                goal_pose.orientation.y = 0.0

            for waypoint in self.waypoints_world:
                waypoints.append(
                    self._convert_to_waypoint(waypoint).autoware_from_world_coords()
                )

            self.route_node.publish_route(goal_pose, waypoints)
            self.sent_route = True
            logger.info("Route published to Autoware")

        if self.sent_route:
            if self.autoware_state.is_planning():
                self._route_was_calculating = True

            if self.autoware_state.route_set():
                self._route_retry_count = 0
                self._route_was_calculating = False

            elif self.autoware_state.route_rejected(self._route_was_calculating):
                logger.warning(
                    "Route calculation failed: state transitioned 3->2. "
                    f"Retry {self._route_retry_count + 1}/{self.MAX_ROUTE_RETRIES}"
                )
                self._retry_route()

        if (
            self.scenario_loaded
            and self.autoware_state.route_set()
            and not self.autoware_state.sent_engage
        ):
            self.autoware_node.publish_engage(True)
