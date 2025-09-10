#!/usr/bin/env python

import traceback
import os
import importlib
import signal
import sys
import logging
import datetime
import yaml
import json

import multiprocessing

import carla

import time

from srunner.scenariomanager.scenario_manager import ScenarioManager
from srunner.tools.results_manager import ScenarioDefinitionManager
from srunner.scenarios.route_scenario import RouteScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.tools.route_parser import RouteParser
from srunner.tools.environment_parser import EnvironmentParser
from srunner.scenarioconfigs.environment_configuration import EnvironmentConfig
from srunner.scenarioconfigs.route_scenario_configuration import (
    RouteScenarioConfiguration,
)
from srunner.tools import route_manipulation
from srunner.objects.ego_vehicle import EgoVehicle
from srunner.tools.log import LogUtil
from srunner.tools.metrics_collector import MetricsCollector

from srunner.tools.CARLA_manager import CARLAManager

logger = logging.getLogger("scenario-runner")

infractions_dict = {
    "OutsideRouteLanesTest": 0.3,
    "CollisionTest": 1.0,
    "RunningRedLightTest": 0.4,
    "RunningStopTest": 0.25,
    "AgentBlockedTest": 0.4,
}

metrics_collected = {
    "timestamp": 0.0,  # when tick started
    "total_tick": 0.0,
    "scenario_runner_time": 0.0,
    "agent_time": 0.0,
    "latency": 0.0,
    "carla_time": 0.0,
}


class AWScenarioRunner(object):
    # flags
    DEV_MODE = False
    DEBUG = False

    # global class instances
    ego_vehicles = []

    carla_world = None
    carla_client = None

    scenario_manager = None
    definition_manager = None

    aw_agent = None

    def __init__(self, config: dict) -> None:
        """
        Setup Scenario Manager and the Carla client
        """

        self._carla_config = config["carla"]
        self._tm_config = config["traffic_manager"]
        self._scenario_config = config["scenario_runner"]

        # Flags
        self.DEV_MODE = self._scenario_config["dev_mode"]
        self.DEBUG = self._scenario_config["debug"]

        self.curr_iteration = 0
        self.iterations = int(self._scenario_config["algorithm"]["iterations"])

        if not self.DEV_MODE:  # only load agents and algorithms in non-dev mode
            autoware_agent_path = self._scenario_config["agent"]
            module_name = os.path.basename(autoware_agent_path).split(".")[0]
            sys.path.insert(0, os.path.dirname(autoware_agent_path))
            self.module_aw_agent = importlib.import_module(module_name)

            algorithm = self._scenario_config["algorithm"]["path"]
            alg_module = os.path.basename(algorithm).split(".")[0]
            sys.path.insert(0, os.path.dirname(algorithm))
            self.module_algorithm = importlib.import_module(alg_module)

        # main class to execute scenarios
        self.scenario_manager = None

        self.results_manager = ScenarioDefinitionManager()

        #  capture SIGINT for cleanp
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
        result_,
    ) -> None:
        logger.info("Initialising Scenario Manager...")
        self.scenario_manager = ScenarioManager(
            self.DEBUG,
            self._carla_config["sync"],
            self._carla_config["timeout"],
        )

        logger.info("Starting the MetricsCollector thread...")

        MetricsCollector.init_state(
            metrics_collected,
            os.path.join(self.results_manager.last_scenario, "execution_time.txt"),
            include=False,
        )

        logger.info("Connecting to client...")
        self.carla_client = carla.Client(
            self._carla_config["host"], int(self._carla_config["port"])
        )
        self.carla_client.set_timeout(self._carla_config["timeout"])
        CarlaDataProvider.set_client(self.carla_client)

        logger.info("Fetching current world...")

        self.carla_world = self.carla_client.get_world()
        logger.info("Updating map...")
        self.carla_client.load_world(env_config.town)

        logger.info("Updating world settings:")
        # tick asynchronously until then
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self._carla_config["fixed_delta_seconds"]
        self.carla_world.apply_settings(settings)

        logger.info(f"{settings.__str__()}")

        # update the world
        CarlaDataProvider.set_world(self.carla_world)

        logger.info("Spawning ego...")
        ego = EgoVehicle(env_config)
        self.ego_vehicles.append(ego.spawn())
        logger.info("Spawned ego...")

        self.carla_world.tick()  # client must tick to spawn actors

        logger.info("Initialising Autoware...")

        if not self.DEV_MODE:
            logger.info("Loading Autoware agent")
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

        ego.prepare_ego(route[0][0])  # set location to first waypoint

        self.carla_world.tick()
        logger.info("Initialising agent route...")

        # allow the agent to localise and set the route
        budget = int(self._scenario_config["initialisation_budget"])
        status = False  # completion status
        for tick in range(1, budget + 1):
            self.carla_world.tick()
            status = route_config.agent.run_step_init()  # type: ignore

        if not status:
            logger.info("Agent failed to initialise route.")
        else:
            logger.info("Successfully initialised agent; route set.")

        if self._tm_config["active"]:
            logger.info("Loading Traffic Manager...")
            tm_port = int(self._tm_config["port"])  # type: ignore
            CarlaDataProvider.set_traffic_manager_port(tm_port)
            tm = self.carla_client.get_trafficmanager(tm_port)

            tm.set_random_device_seed(int(self._tm_config["seed"]))  # ADD TO CONFIG
            tm.set_synchronous_mode(self._tm_config["sync"])

        try:
            scenario = RouteScenario(
                world=self.carla_world,
                config=route_config,
                debug_mode=self.DEBUG,
                ego_vehicle=ego._actor,
                route=route,
            )
        except Exception:
            logger.info("Could not load Route Scenario")
            traceback.print_exc()

        logger.info("Starting scenario...")
        try:
            self.carla_client.start_recorder("/home/carla/recording.log", True)
            self.scenario_manager.load_scenario(
                scenario, self.aw_agent, follow_ego=self._scenario_config["follow_ego"]
            )
            self.scenario_manager.run_scenario()
            self.carla_client.stop_recorder()
            result = True
        except Exception:
            traceback.print_exc()
            logger.info(
                "Could not load scenario. Please check if the agent class is loading correctly."
            )
            result = False

        # stop the MetricsCollector thread
        MetricsCollector.reset()

        # analyse the scenario
        criteria = self._output_criteria(
            self.scenario_manager.scenario.get_criteria(),  # type: ignore
            f"{self.results_manager.last_scenario}/{scenario_name}.json",
        )

        # update multipprocessing queue
        result_dict = result_.get()
        result_dict["status"] = result
        result_dict["criteria"] = criteria
        result_.put(result_dict)

    def run(self) -> None:
        """The Scenario loop. Read the scenario configuration from the parsed XML files,
        configure the scenario in CARLA and execute. Use the results to and parse to the algorithm callback.
        Repeats **iterations** times, as defined in config.yaml

        """

        logger.info("Loading the initial scenario configuration")

        scenario_name = os.path.split(
            os.path.splitext(self._scenario_config["json"])[0]
        )[1]

        try:
            with open(self._scenario_config["json"], "r", encoding="UTF-8") as raw_json:  # type: ignore
                self.json_definition = json.loads(raw_json.read())
        except json.JSONDecodeError:
            logger.error("Failed to decode scenario defintion, exiting...")
            sys.exit(1)

        # load the algorithm instance
        if not self.DEV_MODE:
            alg_class_name = self.module_algorithm.__name__.title().replace("_", "")
            logger.info(
                f"Loading the algorithm class: f{getattr(self.module_algorithm, alg_class_name)}"
            )
            self.algorithm = getattr(self.module_algorithm, alg_class_name)(
                self._scenario_config["algorithm"]["args"]
            )

        for iteration in range(self.iterations):
            logger.info("Starting CARLA container....")
            CARLAManager.restart_carla()
            time.sleep(5)  # allow CARLA to load

            self.curr_iteration = iteration
            logger.info(f"Starting algorithm iteration number {self.curr_iteration}")

            # parse the initial scenario config
            self.results_manager.parse_json(
                self.json_definition,
                scenario_name,
                str(self.curr_iteration),
                save_def=True,
            )

            env_config = EnvironmentParser.parse_scenario_env(
                os.path.join(self.results_manager.last_scenario, "scenario.xml")
            )

            route_config = RouteParser.parse_routes_file(
                self.results_manager.last_scenario, env_config
            )[self._scenario_config["route_id"]]

            logger.info("Starting scenario in new process...")

            result_dict = {"status": False, "criteria": {}}
            scenario_result = multiprocessing.Queue()
            scenario_result.put(result_dict)

            scenario_process = multiprocessing.Process(
                target=self.run_scenario,  # need to catch connection exception
                args=(
                    route_config,
                    env_config,
                    scenario_name,
                    scenario_result,
                ),
            )

            scenario_process.start()
            scenario_process.join()
            logger.info(f"Scenario iteration {self.curr_iteration} has concluded")

            result = scenario_result.get()

            # kill the scenario process
            if scenario_process.is_alive():
                scenario_process.kill()

            # copy over the recording from CARLA container if env variable is setup
            CARLAManager.fetch_file(
                "/home/carla/recording.log",
                self.results_manager.last_scenario,
            )

            logger.info("Calculating driving score...")
            driving_score = self._calculate_driving_score(result["criteria"])
            logger.info(
                f"Scenario iteration {iteration} achieved a score of {driving_score}"
            )

            # clean up - delete XML files
            self.results_manager.cleanup_xml()

            # read the scenario definition
            if not self.DEV_MODE:
                self.json_definition = self.algorithm._scenario_callback(
                    self.json_definition, driving_score
                )

    def _output_criteria(
        self, criteria, file_name: str, save_file: bool = True
    ) -> dict:
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

        # Save the criteria dictionary into a .json file
        if save_file:
            with open(file_name, "w", encoding="utf-8") as fp:
                json.dump(criteria_dict, fp, sort_keys=False, indent=4)
        return criteria_dict

    def _calculate_driving_score(self, criteria: dict) -> float:
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
        if self.carla_client is not None:
            del self.carla_client
        if self.carla_world is not None:
            del self.carla_world

    def _cleanup(self) -> None:
        """Cleanup function. Removes instances of the CARLA client and WORLD, also destroys the Ego vehicle in CARLA."""
        # Simulation still running and in synchronous mode?
        if self.carla_world is not None:
            try:
                # Reset to asynchronous mode
                self.carla_client.get_trafficmanager(
                    int(self._tm_config["port"])
                ).set_synchronous_mode(False)
            except RuntimeError:
                sys.exit(-1)

        self.scenario_manager.cleanup()

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
            self.aw_agent.destroy()
            self.aw_agent = None


def main():
    config = None
    with open("config.yaml", "r") as stream:
        config = yaml.safe_load(stream)

    log_config = config["log"]

    logger.setLevel(logging.INFO)

    log_formatter = logging.Formatter(log_config["log_format"])

    log_path = LogUtil.create_log_file(log_config["path"])
    fh = logging.FileHandler(log_path, encoding="utf-8")
    sh = logging.StreamHandler(sys.stdout)

    fh.setFormatter(log_formatter)
    sh.setFormatter(log_formatter)

    logger.addHandler(fh)
    logger.addHandler(sh)

    CARLAManager._load_config(config["carla"])

    # reload world and sync must be present when running agent-based route scenarios
    scenario_runner = None
    try:
        scenario_runner = AWScenarioRunner(config)
        results = scenario_runner.run()
        logger.info(results)
    except Exception:  # NOT GOOD PRACTICE PROBABLY CHANGE
        traceback.print_exc()
    finally:
        if scenario_runner is not None:
            scenario_runner.destroy()
            del scenario_runner
    return


if __name__ == "__main__":
    main()
    sys.exit(1)
