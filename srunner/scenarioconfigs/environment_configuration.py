#!/bin/bash

# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla

from srunner.objects.sensors import DefaultSensor

"""
This module provides support to configure the environment for the scenario loop
"""


class EnvironmentConfig(object):
    """
    Simple object to store information about the initial environment setup for the scenario loop
    """

    def __init__(self) -> None:
        self.town: str = ""
        self.ego_model: str = ""
        self.ego_name: str = ""
        self.background_behaviour: bool = False
        self.ego_spawn: carla.Transform | None = None
        self.sensor_config: list[DefaultSensor] = []
        self.route_id: int = 0
        self.iteration: int = 0
