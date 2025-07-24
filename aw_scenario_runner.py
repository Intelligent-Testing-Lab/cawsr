#!/usr/bin/env python

import traceback
import os
import importlib
import signal
import sys
import time
import logging
import datetime
import yaml

import carla

from srunner.scenariomanager.scenario_manager import ScenarioManager
from srunner.scenario_decoder.json_to_xml_files import XMLToFiles
from srunner.tools.results_manager import ResultsManager
from srunner.scenarios.route_scenario import RouteScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.tools.route_parser import RouteParser
from srunner.tools.environment_parser import EnvironmentParser

from srunner.tools.log import LogUtil


class AWScenarioRunner(object):
    ego_vehicles = []

    # world and scenario handlers
    carla_world = None
    carla_client = None

    scenario_manager = None
    scenario_decoder = None
    results_manager = None

    wait_for_update = False
    finished = False

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

        # update the client
        CarlaDataProvider.set_client(self.carla_client)

        # load autoware agent
        # only load if in docker environment
        # debug
        if self._scenario_config["in_docker"]:
            autoware_agent_path = "srunner/autoagents/autoware_agent"
            module_name = os.path.basename(autoware_agent_path).split(".")[0]
            sys.path.insert(0, os.path.dirname(autoware_agent_path))
            self.module_aw_agent = importlib.import_module(module_name)

        # main class to execute scenarios
        self.scenario_manager = ScenarioManager(
            self._scenario_config["debug"],
            self._carla_config["sync"],
            self._carla_config["timeout"],
        )

        self.results_manager = ResultsManager()

        # Create signal handler for SIGINT
        self._shutdown_requested = False
        if sys.platform != "win32":
            signal.signal(signal.SIGHUP, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._start_wall_time = datetime.datetime.now()

        # parse the JSON scenario file
        if self.scenario_decoder is None:
            self.scenario_decoder = XMLToFiles()

        scenario_name = os.path.split(
            os.path.splitext(self._scenario_config["json"])[0]
        )[1]
        self._parse_json(self._scenario_config["json"], scenario_name, "0")

    def _parse_json(self, json: str, scenario: str, iteration: str) -> None:
        """Parses a given JSON Scenario definition. Outputs two XML files used by scenario runner

        Args:
            json (str): filepath to JSON scenario definition
            scenario (str): Name of the scenario
            iteration (str): ID of the scenario. Can be anything, but must be unique
        """
        # create run directory if doesn't exist
        if not self.results_manager.results_path:
            self.results_manager.create_run_folder()

        self.results_manager.create_scenario_folder(
            scenario, iteration, self.results_manager.results_path
        )

        self.scenario_decoder.parse_scenario(json, self.results_manager.last_scenario)

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

    def run_scenario(self, config) -> bool:
        # find the ego vehicle by name
        # only supports one ego
        self.carla_world = self.carla_client.get_world()
        self.carla_client.load_world("Town01")

        ego_missing = True
        while ego_missing:
            self.ego_vehicles = []
            for ego in config.ego_vehicles:
                carla_vehicles = (
                    self.carla_client.get_world().get_actors().filter("vehicle.*")
                )

                for carla_vehicle in carla_vehicles:
                    if carla_vehicle.attributes["role_name"] == ego["name"]:
                        self.ego_vehicles.append(carla_vehicle)
                        ego_missing = False
                        break
                print("Can't find ego, waiting...")
                time.sleep(1)
        print("Found ego")

        # update carla provider
        CarlaDataProvider.set_client(self.carla_client)
        CarlaDataProvider.set_world(self.carla_world)

        self.carla_world.wait_for_tick()

        if self._scenario_config["in_docker"]:
            print("Loading Autoware agent")
            agent_class_name = self.module_aw_agent.__name__.title().replace("_", "")
            try:
                print(getattr(self.module_aw_agent, agent_class_name))
                self.aw_agent = getattr(self.module_aw_agent, agent_class_name)("")
                config.agent = self.aw_agent
            except Exception as e:  # Forces the simulation to run synchronously # pylint: disable=broad-except
                traceback.print_exc()
                print("Could not setup required agent due to {}".format(e))
                # self._cleanup()
                return False

        # ADD TRAFFIC MANAGER SEED TO CONFIG
        tm_port = int(self._args.traffic_port)  # type: ignore
        CarlaDataProvider.set_traffic_manager_port(tm_port)
        tm = self.carla_client.get_trafficmanager(tm_port)
        tm.set_random_device_seed(1)  # ADD TO CONFIG

        tm.set_synchronous_mode(True)

        print("Preparing ego...")

        # update ego position to one specified in route
        ego_transform = config.ego_vehicles[0]["transform"]
        self.ego_vehicles[0].set_transform(ego_transform)
        self.ego_vehicles[0].set_target_velocity(carla.Vector3D())
        self.ego_vehicles[0].set_target_angular_velocity(carla.Vector3D())
        CarlaDataProvider.register_actor(self.ego_vehicles[0], ego_transform)

        print("Loading route...")
        try:
            scenario = RouteScenario(
                world=self.carla_world, config=config, debug_mode=True
            )
        except Exception:
            print("Could not load Route Scenario")
            traceback.print_exc()
            return False

        print("Starting scenario...")
        try:
            self.scenario_manager.load_scenario(scenario, self.aw_agent)
            self.scenario_manager.run_scenario()
            result = True
        except Exception:
            traceback.print_exc()
            print("It doesn't wokr")
            result = False
        return result

    def _load_route_scenario(self) -> None:
        env_config = EnvironmentParser.parse_scenario_env(
            os.path.join(self.results_manager.last_scenario, "scenario.xml")
        )

        route_config = RouteParser.parse_routes_file(
            self.results_manager.last_scenario, env_config.route_id
        )  # type: ignore

        return route_config[0]

    def run(self) -> None:
        # load the route config
        # load the scenarion
        # run it
        # get the metrics
        # call the algorithm callback

        config = self._load_route_scenario()

        # setup CARLA settings
        if self._carla_config["sync"]:
            settings = self.carla_world.get_settings()
            settings.synchonous_mode = True
            settings.fixed_delta_seconds = self._carla_config["fixed_delta_seconds"]
            self.carla_world.apply_settings(settings)

        scenario_result = self.run_scenario(config)
        return scenario_result

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
                    int(self._args.traffic_port)
                ).set_synchronous_mode(False)
            except RuntimeError:
                sys.exit(-1)

        self.scenario_manager.cleanup()

        CarlaDataProvider.cleanup()

        for i, _ in enumerate(self.ego_vehicles):
            if self.ego_vehicles[i]:
                if self.ego_vehicles[i] is not None and self.ego_vehicles[i].is_alive:
                    print("Destroying ego vehicle {}".format(self.ego_vehicles[i].id))
                    self.ego_vehicles[i].destroy()
                self.ego_vehicles[i] = None
        self.ego_vehicles = []

        if self.aw_agent:
            self.aw_agent.destroy()
            self.aw_agent = None


def main():
    # configure logger
    config = None
    with open("config.yaml", "r") as stream:
        config = yaml.safe_load(stream)

    log_config = config["log"]

    logger = logging.getLogger("scenario-runner")
    logger.setLevel(logging.INFO)

    log_path = LogUtil.create_log_file(log_config["path"])
    logger.addHandler(logging.FileHandler(log_path, encoding="utf-8"))
    logger.addHandler(logging.StreamHandler(sys.stdout))

    # reload world and sync must be present when running agent-based route scenarios
    scenario_runner = None
    try:
        scenario_runner = AWScenarioRunner(config)
        results = scenario_runner.run()
        print(results)
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
