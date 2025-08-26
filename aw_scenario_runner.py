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

import carla


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


from srunner.objects.ego_vehicle import EgoVehicle

from srunner.tools.log import LogUtil


logger = logging.getLogger("scenario-runner")


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

        self.carla_client = carla.Client(
            self._carla_config["host"], int(self._carla_config["port"])
        )

        self.carla_client.set_timeout(self._carla_config["timeout"])

        # Flags
        self.DEV_MODE = self._scenario_config["dev_mode"]
        self.DEBUG = self._scenario_config["debug"]

        self.curr_iteration = 0
        self.iterations = int(self._scenario_config["algorithm"]["iterations"])

        CarlaDataProvider.set_client(self.carla_client)

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
        self.scenario_manager = ScenarioManager(
            self.DEBUG,
            self._carla_config["sync"],
            self._carla_config["timeout"],
        )

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
        self, route_config: RouteScenarioConfiguration, env_config: EnvironmentConfig
    ) -> bool:
        self.carla_world = self.carla_client.get_world()
        self.carla_client.load_world(env_config.town)

        logger.info("Updating world settings:")
        # tick asynchronously until then
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self._carla_config["fixed_delta_seconds"]
        self.carla_world.apply_settings(settings)

        logger.info(f"{settings.__str__()}")

        if self._tm_config["active"]:
            logger.info("Loading Traffic Manager...")
            tm_port = int(self._tm_config["port"])  # type: ignore
            CarlaDataProvider.set_traffic_manager_port(tm_port)
            tm = self.carla_client.get_trafficmanager(tm_port)

            tm.set_random_device_seed(int(self._tm_config["seed"]))  # ADD TO CONFIG
            tm.set_synchronous_mode(self._tm_config["sync"])

        # update the world
        CarlaDataProvider.set_world(self.carla_world)

        logger.info("Spawning ego...")
        ego = EgoVehicle(env_config)
        self.ego_vehicles.append(ego.spawn())
        logger.info("Spawned ego...")

        self.carla_world.tick()  # client must tick to spawn actors

        logger.info("Setting up sensor configuration...")
        ego.setup_sensors()

        self.carla_world.tick()

        if not self.DEV_MODE:
            logger.info("Loading Autoware agent")
            agent_class_name = self.module_aw_agent.__name__.title().replace("_", "")
            try:
                logger.info(getattr(self.module_aw_agent, agent_class_name))
                self.aw_agent = getattr(self.module_aw_agent, agent_class_name)(
                    env_config
                )
                route_config.agent = self.aw_agent
            except Exception as e:  # Forces the simulation to run synchronously # pylint: disable=broad-except
                logger.error("Could not setup required agent due to {}".format(e))
                self._cleanup()
                return False

        ego.prepare_ego()

        logger.info("Loading route...")

        try:
            scenario = RouteScenario(
                world=self.carla_world,
                config=route_config,
                debug_mode=self.DEBUG,
                ego_vehicle=ego._actor,
            )
        except Exception:
            logger.info("Could not load Route Scenario")
            traceback.print_exc()
            return False

        logger.info("Starting scenario...")
        try:
            recorder_name = f"{self.results_manager.last_scenario}/recording.log"
            self.client.start_recorder(recorder_name, True)

            self.scenario_manager.load_scenario(scenario, self.aw_agent)
            self.scenario_manager.run_scenario()

            self.client.stop_recorder()
            result = True
        except Exception:
            traceback.print_exc()
            logger.info("It doesn't work")
            result = False
        return result

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
        alg_class_name = self.module_algorithm.__name__.title().replace("_", "")
        logger.info(
            f"Loading the algorithm class: f{getattr(self.module_algorithm, alg_class_name)}"
        )
        self.algorithm = getattr(self.module_algorithm, alg_class_name)(
            self._scenario_config["algorithm"]["args"]
        )

        for iteration in range(self.iterations):
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

            self.run_scenario(route_config, env_config)

            # run the metric manager with the recorded file to calculate the driving score
            driving_score = 0.0

            # read the scenario definition
            self.json_definition = self.algorithm._scenario_callback(
                self.json_definition, driving_score
            )

            # destroy the agent to be loaded again
            if self.aw_agent:
                self.aw_agent.destroy()
                self.aw_agent = None

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
    # single argument of configuration file

    # configure logger
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
