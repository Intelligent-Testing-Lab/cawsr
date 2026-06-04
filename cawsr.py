#!/usr/bin/env python

# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import traceback
import os
import importlib
import signal
import sys
import logging
import datetime
import yaml
import pathlib
import random
import json

import multiprocessing

import carla

import time
import numpy as np  # type: ignore

from typing import Optional, Union, Callable

from srunner.scenariomanager.scenario_manager import ScenarioManager
from srunner.scenariomanager.timer import GameTime
from srunner.tools.results_manager import ScenarioDefinitionManager
from srunner.scenarios.route_scenario import RouteScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.tools.route_parser import RouteParser
from srunner.tools.environment_parser import EnvironmentParser
from srunner.scenarioconfigs.environment_configuration import EnvironmentConfig
from srunner.scenarioconfigs.route_scenario_configuration import (
    RouteScenarioConfiguration,
)
from srunner.tools.route_visualisation import visualise_route
from srunner.tools import route_manipulation
from srunner.objects.ego_vehicle import EgoVehicle
from srunner.tools.log import LogUtil
from srunner.tools.metrics_collector import MetricsCollector
from srunner.scenarioconfigs.carla_config import CARLA
from srunner.tools.CARLA_manager import CARLAManager

from algorithms.basic_algorithm import BasicAlgorithm

logger = logging.getLogger("scenario-runner")

metrics_collected = {
    "timestamp": 0.0,
    "total_tick": 0.0,
    "scenario_runner_time": 0.0,
    "agent_time": {
        "sensor": 0.0,
        "control": 0.0,
        "agent_total": 0.0,
    },
    "latency": 0.0,
    "carla_time": 0.0,
}


class AWScenarioRunner(object):
    aw_agent = None  # autoware agent

    def __init__(self, cawsr_config: dict, carla_conf: CARLA) -> None:
        """Initialises CAWSR, loading the Autoware agent module and setting up signal handlers for cleanup on shutdown.
        Also initialises the ScenarioDefinitionManager for managing results directories and files.

        Args:
            cawsr_config (dict): CAWSR configuration options loaded from YAML file
            carla_conf (CARLA): CARLA-specific configuration options loaded from YAML file
        """
        self.DEBUG = cawsr_config["debug"]
        self._conf = cawsr_config
        self._carla = carla_conf

        self.ego_vehicles = []

        # load autoware agent module
        autoware_agent_path = self._conf["agent"]
        module_name = os.path.basename(autoware_agent_path).split(".")[0]
        sys.path.insert(0, os.path.dirname(autoware_agent_path))
        self.module_aw_agent = importlib.import_module(module_name)

        if str(self._conf["mode"]).lower() == "algorithm":
            self._rng = np.random.default_rng(self._conf["algorithm"]["seed"])

        # manages results directories
        self.results_manager = ScenarioDefinitionManager()

        #  capture SIGINT for cleanup
        self._shutdown_requested = False

        if sys.platform != "win32":
            signal.signal(signal.SIGHUP, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._start_wall_time = datetime.datetime.now()

    def _signal_handler(self, signum, frame) -> None:
        """
        Handle shutdown signal, do cleanup
        """
        self._shutdown_requested = True
        if self.scenario_manager:
            self.scenario_manager.stop_scenario()
            self.destroy()
            if not self.scenario_manager.get_running_status():
                raise RuntimeError("Scenario Timeout")

    def run_scenario(
        self,
        route_config: RouteScenarioConfiguration,
        env_config: EnvironmentConfig,
        scenario_name: str,
        seed: int,
        algorithm_mode: bool,
        result_,
        current_definiton: Union[dict, None],
        algorithm: Union[Callable, None],
    ) -> None:
        """Starts the CAWSR scenario based on the provided configuration and seed
        Handles connection to the Autoware agent and CARLA client, and runs the scenario manager until the scenario is complete or a shutdown signal is received.
        Args:
            route_config (RouteScenarioConfiguration): RouteSCenarioConfiguration object configuration
            env_config (EnvironmentConfig): EnvironmentConfig object containing the environment configuration
            scenario_name (str): Name of the scenario to run
            seed (int): Random seed for reproducibility
            algorithm_mode (bool): Whether to load algorithmic scenario generation loop, or just execute scenarios in sequence
            result_ (_type_): Result object for storing scenario results
            current_definiton (Union[dict, None]): Optional dict containing the current scenario definition, used for algorithmic scenario generation
            algorithm (Union[Callable, None]): Optional Callable algorithm class, used for algorithmic scenario generation. See docs for expected interface.
        """
        logger.info("Initialising Scenario Manager...")
        self.scenario_manager = ScenarioManager(
            self.DEBUG,
            self._carla.SYNC,
            self._carla.TIMEOUT,
        )

        logger.info("Starting the MetricsCollector thread...")

        MetricsCollector.init_state(
            metrics_collected,
            os.path.join(self.results_manager.last_scenario, "execution_time.txt"),
            include=False,
        )

        logger.info("Connecting to client...")
        self.carla_client = carla.Client(self._carla.HOST, self._carla.PORT)
        self.carla_client.set_timeout(self._carla.TIMEOUT)
        CarlaDataProvider.set_client(self.carla_client)

        logger.info("Fetching current world...")

        self.carla_world = self.carla_client.get_world()
        logger.info("Updating map...")
        self.carla_client.load_world(env_config.town)

        logger.info("Updating world settings:")

        # tick asynchronously until then
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self._carla.FIXED_DELTA_SECONDS
        self.carla_world.apply_settings(settings)

        logger.info(f"{settings.__str__()}")

        logger.info("Restarting GameTime...")
        GameTime.restart()

        # update the world
        CarlaDataProvider.set_world(self.carla_world)

        # attempt to spawn the ego 3 times for redundancy
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                logger.info("Spawning ego...")
                ego = EgoVehicle(env_config)
                actor = ego.spawn()
                self.ego_vehicles.append(actor)
                logger.info(f"Spawned ego with id: {actor.id}")
                break  # exit loop if successful
            except Exception as e:
                logger.error(f"Failed to spawn ego on attempt {attempt + 1}: {e}")
                if attempt == max_attempts - 1:
                    logger.error(
                        "Failed to spawn ego: Terminating scenario. This is most likely an issue with CARLA"
                    )
                    raise
                time.sleep(2)

        # client must tick to spawn actors
        self._tick_carla()

        logger.info("Initialising Autoware...")
        agent_class_name = self.module_aw_agent.__name__.title().replace("_", "")
        try:
            logger.info(getattr(self.module_aw_agent, agent_class_name))
            self.aw_agent = getattr(self.module_aw_agent, agent_class_name)(
                env_config
            )  # agent init function
            route_config.agent = self.aw_agent
        except Exception as e:  # Forces the simulation to run synchronously # pylint: disable=broad-except
            logger.error("Could not setup required agent due to {}".format(e))
            self._cleanup()
            result = False
            return

        logger.info("Loading route...")

        gps_route, route = route_manipulation.interpolate_trajectory(
            route_config.keypoints
        )
        route_config.agent.set_global_plan(gps_route, route)  # set agent route

        # visualise the route in CARLA and set location to first waypoint
        visualise_route([waypoint[0] for waypoint in route])
        ego.prepare_ego(route[0][0])

        # allow the agent X ticks to initialize sensors and set the route
        # uses the same tick pattern as the scenario_manager main loop
        logger.info("Initialising agent route...")
        budget = self._conf["initialisation_budget"]
        max_route_attempts = 3
        route_ready = False

        for attempt in range(1, max_route_attempts + 1):
            if attempt > 1:
                logger.info(
                    f"Retrying agent route setup ({attempt}/{max_route_attempts})"
                )
                self.aw_agent.agent_set_route = False
                self.aw_agent.sent_route = False
                self.aw_agent.autoware_state.sent_route = False
                self.aw_agent.route_node.request_clear_route()

            for tick in range(1, budget + 1):
                _tick_carla_start = time.perf_counter_ns() / 1e6
                world = CarlaDataProvider.get_world()
                if world:
                    world.tick()
                    for _ in range(5):
                        time.sleep(0)
                MetricsCollector.update_key(
                    "carla_time", (time.perf_counter_ns() / 1e6) - _tick_carla_start
                )
                timestamp = None
                if world:
                    snapshot = world.get_snapshot()
                    if snapshot:
                        timestamp = snapshot.timestamp
                if timestamp:
                    self.aw_agent._carla_timestamp = timestamp
                    GameTime.on_carla_tick(timestamp)
                    CarlaDataProvider.on_carla_tick()
                    self.aw_agent.run_step()
                if self.aw_agent.autoware_state.route_set():
                    route_ready = True
                    break

            if route_ready:
                break

            logger.info(f"Agent failed to initialise route on attempt {attempt}")

        if not route_ready:
            logger.error(
                "Agent failed to initialise route after 3 attempts; skipping scenario."
            )
            result_dict = result_.get()
            result_dict["status"] = False
            result_dict["driving_score"] = 0.0

            if algorithm_mode:
                algorithm._update_generator(seed)  # type: ignore
                definition = algorithm._scenario_callback(  # type: ignore
                    current_definiton, 0.0
                )
                result_dict["definition"] = definition

            result_.put(result_dict)
            MetricsCollector.reset()
            return

        logger.info("Loading Traffic Manager...")
        tm_port = int(self._carla.TRAFFIC_MANAGER.PORT)  # type: ignore
        CarlaDataProvider.set_traffic_manager_port(tm_port)
        tm = self.carla_client.get_trafficmanager(tm_port)

        tm.set_random_device_seed(int(self._carla.TRAFFIC_MANAGER.SEED))
        tm.set_synchronous_mode(self._carla.SYNC)

        try:
            scenario = RouteScenario(
                world=self.carla_world,
                config=route_config,
                debug_mode=self.DEBUG,
                timeout=self._conf.get("route_timeout", None),
                ego_vehicle=ego._actor,
                route=route,
            )
        except Exception:
            logger.info("Could not load Route Scenario")
            traceback.print_exc()

        logger.info("Starting scenario...")

        self.aw_agent.scenario_loaded = True

        try:
            self.carla_client.start_recorder(
                "/home/carla/recordings/recording.log", True
            )
            self.scenario_manager.load_scenario(
                scenario, self.aw_agent, follow_ego=True
            )

            self.scenario_manager.run_scenario()
            result = True
        except Exception:
            traceback.print_exc()
            logger.info(
                "Could not load scenario. Please check if the agent class is loading correctly."
            )
            result = False
        finally:
            self.carla_client.stop_recorder()

        # stop the MetricsCollector thread
        MetricsCollector.reset()

        try:
            # analyse the scenario, throws exception if scenario didn't finish
            criteria = self._output_criteria(
                self.scenario_manager.scenario.get_criteria(),  # type: ignore
                f"{self.results_manager.last_scenario}/{scenario_name}.json",
            )
            logger.info("Calculating driving score...")

            result_dict = result_.get()
            result_dict["driving_score"] = self._calculate_driving_score(criteria)
            result_dict["status"] = result

        except Exception:
            logger.info("Something went wrong, retrying scenario...")

        # read the scenario definition
        if algorithm_mode:
            algorithm._update_generator(seed)  # type: ignore

            try:
                definition = algorithm._scenario_callback(  # type: ignore
                    current_definiton, result_dict["driving_score"]
                )
                result_dict["definition"] = definition
                result_.put(result_dict)
            except Exception:
                logger.error(
                    "Something went wrong while processing algorithm callback; is CARLA alive?"
                )

    def _tick_carla(self) -> None:
        """Advances CARLA 1 tick into the future"""
        world = CarlaDataProvider.get_world()
        if world:
            world.tick()
            for _ in range(5):
                time.sleep(0)

            snapshot = world.get_snapshot()
            if snapshot:
                timestamp = snapshot.timestamp
                if self.aw_agent is not None:
                    self.aw_agent._carla_timestamp = timestamp
                GameTime.on_carla_tick(timestamp)
                CarlaDataProvider.on_carla_tick()

    def _load_alg(self) -> type[BasicAlgorithm]:
        """Load an algorithm instance from mounted docker volume algorithms/

        Returns:
            _type_: Callable algorithm class
        """
        algorithm = self._conf["algorithm"]["path"]
        alg_module = os.path.basename(algorithm).split(".")[0]
        sys.path.insert(0, os.path.dirname(algorithm))
        self.module_algorithm = importlib.import_module(alg_module)

        alg_class_name = self.module_algorithm.__name__.title().replace("_", "")
        logger.info(
            f"Loading the algorithm class: f{getattr(self.module_algorithm, alg_class_name)}"
        )
        return getattr(self.module_algorithm, alg_class_name)(
            self._conf["algorithm"]["args"]
        )

    def run_algorithm(self) -> None:
        """Executes CAWSR in algorithm mode, loading a custom algorithm interface for scenario generation and optimisation.
        Scenarios are generated iteratively based on the previous scenario's performance, and the initial scenario definition provided.
        """

        # load the algorithm and scenario definitions
        optimisation_algorithm = self._load_alg()
        scenario = pathlib.Path(self._conf["algorithm"]["initial_definition"])

        logger.info(str(scenario.absolute()))
        json_definition = self._load_scenario(
            scenario_name=scenario.stem,
            path=str(scenario.absolute()),
            run=0,
            save_def=True,
            return_def=True,
        )

        runs = self._conf["algorithm"]["runs"]
        for i in range(0, runs):
            logger.info(f"Running scenario {i + 1}/{runs}")

            CARLAManager._set_recording_dir(self.results_manager.recording_path)
            logger.info("Starting CARLA container....")
            CARLAManager.restart_carla()
            time.sleep(10)  # allow CARLA to load

            env_config = EnvironmentParser.parse_scenario_env(
                self.results_manager.fetch_scenario_xml()
            )
            route_config = RouteParser.parse_routes_file(
                self.results_manager.last_scenario, env_config
            )[
                0
            ]  # route id. Multiple routes currently aren't supported, so use first route

            result = self._cawsr_process(
                route_config=route_config,
                env_config=env_config,
                scenario_name=scenario.stem,
                run=i + 1,
                algorithm_mode=True,
                algorithm=optimisation_algorithm,
                current_definiton=json_definition,
            )

            self.results_manager.parse_json(
                result["definition"],  # type: ignore
                scenario.stem,
                str(i),
                save_def=True,
            )

    def run_benchmark(self) -> None:
        """Executes CAWSR in benchmark mode. Scenarios are loaded from the target directory, and either executed in sequence or randomly sampled based on a seed."""
        target_dir = pathlib.Path(self._conf["benchmark"]["scenarios"])
        scenarios = [scenario for scenario in target_dir.glob("*.json")]

        runs = len(scenarios)
        for i in range(0, runs):
            retry_attempts = 2
            attempts = 0

            while attempts < retry_attempts:
                logger.info(f"Running scenario {i + 1}/{runs}")

                if self._conf["benchmark"]["random_sampling"]:
                    scenario = random.choice(scenarios)
                    scenarios.remove(
                        scenario
                    )  # each scenario can only be executed once
                else:
                    scenario = scenarios[i]

                self._load_scenario(
                    scenario_name=scenario.stem,
                    path=str(scenario.absolute()),
                    run=i,
                    save_def=True,
                )

                CARLAManager._set_recording_dir(self.results_manager.recording_path)
                logger.info("Starting CARLA container....")
                CARLAManager.restart_carla()
                time.sleep(10)  # allow CARLA to load

                env_config = EnvironmentParser.parse_scenario_env(
                    self.results_manager.fetch_scenario_xml()
                )
                route_config = RouteParser.parse_routes_file(
                    self.results_manager.last_scenario, env_config
                )[
                    0
                ]  # route id. Multiple routes currently aren't supported, so use first route

                result = self._cawsr_process(
                    route_config=route_config,
                    env_config=env_config,
                    scenario_name=scenario.stem,
                    run=i + 1,
                    algorithm_mode=False,
                )

                # scenario failed, run it again.
                if not result["status"] and result["driving_score"] is None:
                    attempts += 1
                else:
                    attempts = (
                        retry_attempts  # successful execution, move onto the next
                    )

    def _cawsr_process(
        self,
        route_config: RouteScenarioConfiguration,
        env_config: EnvironmentConfig,
        scenario_name: str,
        run: int,
        algorithm_mode: bool = False,
        current_definiton: Union[dict, None] = None,
        algorithm: Union[Callable, None] = None,
    ) -> dict:
        """Spawns a CAWSR process to execute a given, configured scenario.
        Handles setup, execution, and teardown of the scenario, and returns the scenario definition if in algorithm mode for use in the next iteration of the algorithm.

        Args:
            route_config (RouteScenarioConfiguration): Scenario configuration object containing the route and agent information
            env_config (EnvironmentConfig): Scenario configuration object containing the environment information, such as weather and town
            scenario_name (str): Name of the scenario, used for results management and logging
            run (int): Current run number, used for logging and results management
            algorithm_mode (bool, optional): Whether to run in algorithm mode. Defaults to False.
            current_definiton (Union[dict, None], optional): The current scenario definition. Defaults to None.
            algorithm (Union[Callable, None], optional): The algorithm to use. Defaults to None.

        Returns:
            Optional[dict]: The updated scenario definition if in algorithm mode, otherwise None.
        """
        scenario_result = multiprocessing.Queue()
        scenario_result.put({"status": None, "driving_score": None, "criteria": {}})

        if algorithm_mode:
            seed = self._rng.integers(
                0, sys.maxsize
            )  # generate a random seed from the seed
        else:
            seed = 0

        env_config.iteration = run
        scenario_process = multiprocessing.Process(
            target=self.run_scenario,  # need to catch connection exception
            args=(
                route_config,
                env_config,
                scenario_name,
                seed,
                algorithm_mode,
                scenario_result,
                current_definiton,
                algorithm,
            ),
        )
        # run scenario until finished
        logger.info(f"Starting {scenario_name} process.")
        scenario_process.start()
        scenario_process.join()

        # fetch result and clean up
        result = scenario_result.get()

        if scenario_process.is_alive():
            scenario_process.kill()
        self.results_manager.cleanup_xml()

        time.sleep(1)

        # if the process exits (CARLA crash for example), these are none
        # fetch execution status of the scenario (failure or success)
        status = result.get(["status"], None)
        driving_score = result.get(["driving_score"], None)

        logger.info(f"Scenario iteration {run} achieved a score of {driving_score}")
        logger.info(
            f"Scenario iteration {run} ended with status {'SUCCESS' if status else 'FAILURE'}"
        )

        return result

    def _load_scenario(
        self,
        scenario_name: str,
        path: str,
        run: int,
        save_def: bool = True,
        return_def: bool = False,
    ) -> Optional[dict]:
        """Loads a given scenario from the JSON definition file

        Args:
            scenario_name (str): Name of the scenario, used for results management and logging
            path (str): Path to the JSON definition file
            run (int): Current run number, used for logging and results management
            save_def (bool, optional): Whether to save the scenario definition. Defaults to True.
            return_def (bool, optional): Whether to return the scenario definition. Defaults to False.

        Returns:
            Optional[dict]: The loaded scenario definition if `return_def` is True, otherwise None.
        """
        try:
            with open(path, "r", encoding="UTF-8") as raw_json:
                json_definition = json.loads(raw_json.read())
        except FileNotFoundError:
            logger.info("JSON definition does not exist, exiting...")
            sys.exit(1)
        except json.JSONDecodeError:
            logger.error("Failed to decode scenario defintion, exiting...")
            sys.exit(1)

        # parse the config and create the results dir
        logger.info("Parsing JSON definition...")
        self.results_manager.parse_json(
            json_definition,
            scenario_name,
            str(run),
            save_def=save_def,
        )

        if return_def:
            return json_definition

    def run(self) -> None:
        """Starts the scenario loop based on the mode selected"""
        logger.info("Loading the initial scenario configuration")

        if str(self._conf["mode"]).lower() == "algorithm":
            self.run_algorithm()
        elif str(self._conf["mode"]).lower() == "benchmark":
            self.run_benchmark()

    def _output_criteria(
        self, criteria, file_name: str, save_file: bool = True
    ) -> dict:
        """Filter the criteria.json generated by scenario-runner to calculate driving score

        Args:
            criteria (_type_): The criteria generated by scenario-runner, containing all the information about the scenario execution, including the criteria results and their attributes
            file_name (str):  The name of the file to save the filtered criteria to, used for results management and logging
            save_file (bool, optional): Whether to save the filtered criteria to a file. Defaults to True.

        Returns:
            dict: The filtered criteria dictionary
        """
        # Filter the attributes that aren't JSON serializable
        with open("temp.json", "w", encoding="utf-8") as fp:
            criteria_dict = {}
            for criterion in criteria:
                criterion_dict = criterion.__dict__
                criteria_dict[criterion.name] = {}

                for key in criterion_dict:
                    if key != "name":
                        try:
                            key_dict = {key: criterion_dict[key]}
                            json.dump(key_dict, fp, sort_keys=False, indent=4)
                            criteria_dict[criterion.name].update(key_dict)
                        except TypeError:
                            pass

        os.remove("temp.json")

        # save the criteria dictionary into a .json file
        if save_file:
            with open(file_name, "w", encoding="utf-8") as fp:
                json.dump(criteria_dict, fp, sort_keys=False, indent=4)
        return criteria_dict

    def _calculate_driving_score(self, criteria: dict) -> float:
        """Returns driving score based on a set of provided infractions, and the criteria log generated by scenario runner.

        Args:
            criteria (dict): The criteria dictionary generated by scenario runner, containing the results of all criteria and their attributes. See `_output_criteria` for details on how this is generated.

        Returns:
            float: The calculated driving score
        """
        infractions_dict = {
            "OutsideRouteLanesTest": 0.3,
            "CollisionTest": 1.0,
            "RunningRedLightTest": 0.4,
            "RunningStopTest": 0.25,
            "AgentBlockedTest": 0.4,
        }

        driving_score = 0.0

        completed_route = float(criteria["RouteCompletionTest"]["actual_value"]) / 100
        logger.info(f"Agent route completion: {completed_route * 100}%")

        logger.info("Checking penality conditions...")
        penalties = 1
        for infraction, penalty in infractions_dict.items():
            delta_penalty = float(criteria[infraction]["actual_value"] * penalty)

            if delta_penalty:
                logger.info(
                    f"Condition {infraction}: Breached {criteria[infraction]['actual_value']} times"
                )
                logger.info(f"Applying penalty of {delta_penalty}")
            else:
                logger.info(f"Condition {infraction}: Found zero breaches")

            penalties += delta_penalty

        driving_score = completed_route * (1 / penalties)
        return driving_score

    def destroy(self) -> None:
        """Deletes instances of all classes related to CARLA"""

        self._cleanup()
        if self.scenario_manager is not None:
            del self.scenario_manager

    def _cleanup(self) -> None:
        """Cleanup function. Removes instances of the CARLA client and WORLD, also destroys the Ego vehicle in CARLA."""
        CARLAManager.stop_carla()

        try:
            client = CarlaDataProvider.get_client()
            if client is not None:
                client.get_trafficmanager(
                    int(self._carla.TRAFFIC_MANAGER.PORT)
                ).set_synchronous_mode(False)
        except RuntimeError:
            sys.exit(-1)

        if self.scenario_manager is not None:
            self.scenario_manager.cleanup()

        if CarlaDataProvider is not None:
            CarlaDataProvider.cleanup()

        for i, _ in enumerate(self.ego_vehicles):
            if self.ego_vehicles[i]:
                if self.ego_vehicles[i] is not None and self.ego_vehicles[i].is_alive:
                    logger.info(
                        "Destroying ego vehicle {}".format(self.ego_vehicles[i].id)
                    )
                    self.ego_vehicles[i].destroy()
                self.ego_vehicles[i] = None
        self.ego_vehicles = []

        if self.aw_agent:
            self.aw_agent.destroy(cleanup=True)
            self.aw_agent = None


def init_logs() -> None:
    """Sets up the CAWSR logging object. Can be retrieved by calling
    `logginer.get_logger("scenario-runner")`
    """
    logger.setLevel(logging.INFO)

    log_formatter = logging.Formatter(
        "[%(levelname)s] [%(created)f] [%(filename)s] %(message)s"
    )

    log_path = LogUtil.create_log_file("logs/")
    fh = logging.FileHandler(log_path, encoding="utf-8")
    sh = logging.StreamHandler(sys.stdout)

    fh.setFormatter(log_formatter)
    sh.setFormatter(log_formatter)

    logger.addHandler(fh)
    logger.addHandler(sh)


def init_config() -> dict:
    """Load the config file from the passed env variable.

    Returns:
        dict: config dict. Empty if config failed to load
    """
    config_env = os.getenv("CAWSR_CONFIG", "configs/example_algorithm.json")

    try:
        with open(config_env, "r") as stream:
            config = yaml.safe_load(stream)["cawsr"]
        return config
    except FileNotFoundError:
        logger.error("Failed to load config file, does it exist?")
        return {}


def main():
    """Entrypoint"""
    init_logs()

    config = init_config()
    carla_config = CARLA()
    carla_config._parse_dict(carla_config, config["carla"])
    CARLAManager._load_config(carla_config)

    # reload world and sync must be present when running agent-based route scenarios
    scenario_runner = None
    try:
        scenario_runner = AWScenarioRunner(config, carla_config)
        results = scenario_runner.run()
        logger.info(results)
    except Exception:  # catch any anonymous exceptions to ensure cleanup is always called, as CARLA can leave zombie processes
        traceback.print_exc()
    finally:
        if scenario_runner is not None:
            scenario_runner.destroy()
            del scenario_runner
    return


if __name__ == "__main__":
    main()
    sys.exit(1)
