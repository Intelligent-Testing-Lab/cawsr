#!/usr/bin/env python


import traceback
import os
import importlib
import signal
import sys
import time
import argparse
import datetime

import carla

from srunner.scenariomanager.scenario_manager import ScenarioManager
from srunner.scenario_decoder.json_to_xml_files import XMLToFiles
from srunner.tools.results_manager import ResultsManager
from srunner.scenarios.route_scenario import RouteScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.tools.route_parser import RouteParser


class AWScenarioRunner(object):
    client_timeout = 10.0

    ego_vehicles = []

    results_path = None
    last_scenario_path = None
    route_id = None

    # world and scenario handlers
    carla_world = None
    carla_client = None

    scenario_manager = None
    scenario_decoder = None
    results_manager = None

    wait_for_update = False
    finished = False

    sync = False  # CARLA-Bridge will do the ticking

    aw_agent = None

    def __init__(self, args: object) -> None:
        """
        Setup Scenario Manager and the Carla client
        """
        self._args = args

        if args.timeout:  # type: ignore
            self.client_timeout = float(args.timeout)  # type: ignore
        self.carla_client = carla.Client(args.host, int(args.port))  # type: ignore
        self.carla_client.set_timeout(self.client_timeout)  # type: ignore

        # update the client
        CarlaDataProvider.set_client(self.carla_client)

        if args.route_id:  # type:ignore
            self.route_id = str(args.route_id)  # type:ignore
        else:
            self.route_id = "0"

        # load autoware agent
        autoware_agent_path = "srunner/autoagents/autoware_agent"
        module_name = os.path.basename(autoware_agent_path).split(".")[0]
        sys.path.insert(0, os.path.dirname(autoware_agent_path))
        self.module_aw_agent = importlib.import_module(module_name)

        self.scenario_manager = ScenarioManager(
            args.debug,
            self.sync,
            self.client_timeout,  # type: ignore
        )

        self.results_manager = ResultsManager()

        # Create signal handler for SIGINT
        self._shutdown_requested = False
        if sys.platform != "win32":
            signal.signal(signal.SIGHUP, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._start_wall_time = datetime.datetime.now()  #

        # parse the JSON scenario file
        if self.scenario_decoder is None:
            self.scenario_decoder = XMLToFiles()
        self._parse_json(args.json, "route-scenario", 0)  # type: ignore

    def _parse_json(self, json: str, scenario: str, iteration: int) -> None:
        # create a new directory in results
        # like results-[date]-[time]

        # create new results directory
        if not self.results_path:
            self.results_path = self.results_manager._create_run_folder()  # type: ignore

        # first scenario run
        self.last_scenario_path = self.results_manager._create_scenario_folder(
            scenario, iteration, self.results_path
        )  # type: ignore
        # parse the json, saving to scenario run path
        self.scenario_decoder.parse_scenario(json, self.last_scenario_path)  # type: ignore

    def _signal_handler(self, signum, frame) -> None:
        """
        Handle cleanup
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

        print("Loading Autoware agent")
        agent_class_name = self.module_aw_agent.__name__.title().replace("_", "")
        try:
            print(agent_class_name)
            print(getattr(self.module_aw_agent, agent_class_name))
            self.aw_agent = getattr(self.module_aw_agent, agent_class_name)("")
            config.agent = self.aw_agent
            print(config.agent)
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
        # take self.last_scenario_path
        route_config = RouteParser.parse_routes_file(
            self.last_scenario_path, self.route_id
        )  # type: ignore

        return route_config[0]

    def run(self) -> None:
        # load the route config
        # load the scenarion
        # run it
        # get the metrics
        # call the algorithm callback

        config = self._load_route_scenario()
        scenario_result = self.run_scenario(config)
        return scenario_result

        # call the algorithm callback

    def destroy(self) -> None:
        """
        Clean up
        """

        self._cleanup()
        if self.scenario_manager is not None:
            del self.scenario_manager
        if self.carla_client is not None:
            del self.carla_client
        if self.carla_world is not None:
            del self.carla_world

    def _cleanup(self) -> None:
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
    desc = """
        CARLA and Autoware Scenario Runner modified to only use a single scenario definition
        """

    # Avaliable arguments
    #
    #   --port: CARLA port
    #   --host: CARLA host
    #   --alg: custom algorithm class to be imported.
    #   --json: JSON scenario definition
    #   --route-id: ROUTE id to use, specified in the JSON scenario definition
    #   --outputDir: Output directory to create the final results
    #   --no-record: does not record any metrics
    #   --timeout: client timeout

    arg_parser = argparse.ArgumentParser(
        description=desc, formatter_class=argparse.RawTextHelpFormatter
    )

    arg_parser.add_argument("-p", "--port", default=2000, help="CARLA Port")
    arg_parser.add_argument(
        "--traffic-port", default=8000, help="CARLA Traffic Manager Port"
    )
    arg_parser.add_argument("--host", default="127.0.0.1", help="CARLA Host")
    arg_parser.add_argument(
        "-a",
        "--algorithm",
        default=None,
        help="The Algorithm to use when modifying scenarios",
    )
    arg_parser.add_argument(
        "-j", "--json", default=None, help="JSON Scenario description"
    )
    arg_parser.add_argument(
        "-r", "--route-id", default=0, help="Route to use from the SCENARIO definition"
    )
    arg_parser.add_argument(
        "-o",
        "--outputDir",
        default=None,
        help="Specify a custom output directory for the results",
    )
    arg_parser.add_argument(
        "-n", "--no-record", default=None, help="Does not record a scenario replay"
    )
    arg_parser.add_argument(
        "-t", "--timeout", default=None, help="CARLA client timeout"
    )
    arg_parser.add_argument(
        "-d", "--debug", default=False, action="store_true", help="CARLA client timeout"
    )

    arguments = arg_parser.parse_args()

    # reload world and sync must be present when running agent-based route scenarios

    scenario_runner = None
    try:
        scenario_runner = AWScenarioRunner(arguments)
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
