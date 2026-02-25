#!/bin/bash

# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import datetime

import json
from srunner.scenario_decoder.json_to_xml_files import XMLToFiles
from srunner.tools.CARLA_manager import CARLAManager


class ScenarioDefinitionManager(object):
    """Class to manage the file structure of the results. Creates folders for entire experiments (runs) and individual scenario executions (scenarios)
    Holds the file path of the last scenario executes, and the results.

    """

    def __init__(self, output_dir: str = "results") -> None:
        self.output_dir = output_dir
        self.last_scenario = ""
        self.results_path = ""
        self._scenario_decoder = XMLToFiles()

    def fetch_scenario_xml(self) -> str:
        return os.path.join(self.last_scenario, "scenario.xml")

    def _create_run_folder(self, scenario: str, base_path: str = ""):
        """Creates a experiment folder under base_path. The naming convention is as follows
        ```
        run-{yy-mm-dd-h-m-s}
        ```

        Args:
            base_path (str, optional): Root folder. Defaults to 'results'.

        Returns:
            _type_: None
        """
        if not base_path:
            base_path = self.output_dir

        now = datetime.datetime.now()
        full_path = os.path.join(
            base_path, f"{scenario}-{now.strftime('%Y-%m-%d-%H-%M-%S')}"
        )  # type: ignore

        os.makedirs(full_path, exist_ok=True)  # exist_ok=True, no need to error handle
        self.results_path = full_path
        return full_path

    def _create_scenario_folder(self, iteration: str, results_folder: str):
        """Creates a folder to hold the results of a individual scenario execution.
        The naming convention is as follows
        ```
        {results_folder}/{iteration}
        ```

        Args:
            scenario (str): Name of the scenario executed
            iteration (str): ID of the scenario (can by anything, but must be unique)
            results_folder (str): Root folder

        Returns:
            _type_: _description_
        """

        full_path = os.path.join(results_folder, f"{iteration}")

        # make the recordings folder
        os.makedirs(os.path.join(full_path, "recording"), exist_ok=True)
        self.recording_path = os.path.join(full_path, "recording")

        os.makedirs(full_path, exist_ok=True)  # exist_ok=True, no need to error handle
        self.last_scenario = full_path

        # update permissions
        CARLAManager.update_permission(self.last_scenario)
        return full_path

    def parse_json(
        self, json_: str | dict, scenario: str, iteration: str, save_def: bool = False
    ):
        """Parses a given JSON Scenario definition. Outputs two XML files used by scenario runner

        Args:
            json (str): filepath to JSON scenario definition
            scenario (str): Name of the scenario
            iteration (str): ID of the scenario. Can be anything, but must be unique
        """

        if not self.results_path:
            self._create_run_folder(scenario)

        self._create_scenario_folder(iteration, self.results_path)

        # save the definition as a new file
        if save_def and isinstance(json_, dict):
            with open(
                f"{self.last_scenario}/definition.json", "a", encoding="utf-8"
            ) as f:
                json.dump(json_, f)

        self._scenario_decoder.parse_scenario(json_, self.last_scenario)

    def cleanup_xml(self) -> None:
        """Cleanup the .xml files left over by the scenario execution."""

        full_paths = [
            f"{self.last_scenario}/route.xml",
            f"{self.last_scenario}/scenario.xml",
        ]

        for path in full_paths:
            os.remove(path)
