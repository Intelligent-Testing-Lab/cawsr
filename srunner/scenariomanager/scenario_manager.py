#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementation.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import time

import py_trees

import carla

from srunner.autoagents.agent_wrapper import AgentWrapper
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.result_writer import ResultOutputProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog
from srunner.tools.metrics_collector import MetricsCollector


class ScenarioManager(object):
    """
    Basic scenario manager class. This class holds all functionality
    required to start, and analyze a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. Trigger a result evaluation with manager.analyze_scenario()
    5. If needed, cleanup with manager.stop_scenario()
    """

    def __init__(self, debug_mode=False, sync_mode=False, timeout=2.0):
        """
        Setups up the parameters, which will be filled at load_scenario()

        """
        self.scenario = None
        self.scenario_tree = None
        self.ego_vehicles = None
        self.follow_ego = None
        self.other_actors = None

        self._debug_mode = debug_mode
        self._agent = None
        self._sync_mode = sync_mode
        self._watchdog = None
        self._timeout = timeout

        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None

    def _reset(self):
        """
        Reset all parameters
        """
        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        GameTime.restart()

    def cleanup(self):
        """
        This function triggers a proper termination of a scenario
        """

        if self.scenario is not None:
            self.scenario.terminate()

        if self._agent is not None:
            self._agent.cleanup()
            self._agent = None

        if self._watchdog is not None:
            self._watchdog.stop()
            self._watchdog = None

        CarlaDataProvider.cleanup()

    def load_scenario(self, scenario, agent=None, follow_ego=False):
        """
        Load a new scenario
        """
        self._reset()
        self._agent = AgentWrapper(agent) if agent else None
        if self._agent is not None:
            self._sync_mode = True
        self.scenario = scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.follow_ego = follow_ego
        self.other_actors = scenario.other_actors

        if follow_ego:
            self.world_cam = CarlaDataProvider.get_world().get_spectator()
            self._camera_offset = carla.Location(x=-5, y=0, z=15)
            self._camera_pitch = -60.0  # degrees

    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        print("ScenarioManager: Running scenario {}".format(self.scenario_tree.name))
        self.start_system_time = time.time()
        start_game_time = GameTime.get_time()

        self._watchdog = Watchdog(float(self._timeout))
        self._watchdog.start()
        self._running = True

        while self._running:
            _tick_start = time.perf_counter()
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                self._tick_scenario(timestamp)

            MetricsCollector.update_key("timestamp", _tick_start)
            MetricsCollector.update_key("total_tick", time.perf_counter() - _tick_start)

            # calculate latency
            _state = MetricsCollector.fetch_state()
            latency = (
                _state["total_tick"]
                - _state["scenario_runner_time"]
                - _state["agent_time"]
                - _state["carla_time"]
            )

            MetricsCollector.update_key("latency", latency)
            MetricsCollector.save_state()

        self.cleanup()

        self.end_system_time = time.time()
        end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - self.start_system_time
        self.scenario_duration_game = end_game_time - start_game_time

        if self.scenario_tree.status == py_trees.common.Status.FAILURE:
            print("ScenarioManager: Terminated due to failure")

    def _tick_scenario(self, timestamp):
        """
        Run next tick of scenario and the agent.
        If running synchornously, it also handles the ticking of the world.
        """
        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()

            if self._debug_mode:
                print("\n--------- Tick ---------\n")

            _tick_carla_start = time.perf_counter()
            if self._sync_mode and self._watchdog.get_status():
                CarlaDataProvider.get_world().tick()
            MetricsCollector.update_key(
                "carla_time", time.perf_counter() - _tick_carla_start
            )

            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            if self._agent is not None:
                self._agent()  # pylint: disable=not-callable

            # Tick scenario
            _scenario_tick_start = time.perf_counter()
            self.scenario_tree.tick_once()
            MetricsCollector.update_key(
                "scenario_runner_time", time.perf_counter() - _scenario_tick_start
            )

            if self.follow_ego:
                self._tick_spectator_cam(self.ego_vehicles[0])  # type: ignore

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                self._running = False

    def _tick_spectator_cam(self, ego: carla.Actor) -> None:
        """Ticks the spectator camera for the chosen ego"""
        vehicle_transform = ego.get_transform()
        delta_spec_loc = vehicle_transform.location + self._camera_offset

        delta_spec_trans = carla.Transform(
            delta_spec_loc,
            carla.Rotation(
                pitch=self._camera_pitch, yaw=vehicle_transform.rotation.yaw, roll=0
            ),
        )
        self.world_cam.set_transform(delta_spec_trans)

    def get_running_status(self):
        """
        returns:
           bool:  False if watchdog exception occured, True otherwise
        """
        return self._watchdog.get_status()

    def stop_scenario(self):
        """
        This function is used by the overall signal handler to terminate the scenario execution
        """
        self._running = False

    def analyze_scenario(self, stdout, filename, junit, json):
        """
        This function is intended to be called from outside and provide
        the final statistics about the scenario (human-readable, in form of a junit
        report, etc.)
        """

        failure = False
        timeout = False
        result = "SUCCESS"

        criteria = self.scenario.get_criteria()
        if len(criteria) == 0:
            print("Nothing to analyze, this scenario has no criteria")
            return True

        for criterion in criteria:
            if (
                not criterion.optional
                and criterion.test_status != "SUCCESS"
                and criterion.test_status != "ACCEPTABLE"
            ):
                failure = True
                result = "FAILURE"
            elif criterion.test_status == "ACCEPTABLE":
                result = "ACCEPTABLE"

        if self.scenario.timeout_node.timeout and not failure:
            timeout = True
            result = "TIMEOUT"

        output = ResultOutputProvider(self, result, stdout, filename, junit, json)
        output.write()

        return failure or timeout
