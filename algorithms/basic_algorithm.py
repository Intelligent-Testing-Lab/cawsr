#!/bin/bash

# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np


class BasicAlgorithm(object):
    def __init__(self, args: dict) -> None:
        self._args = args

        self._rng = None

    def _scenario_callback(
        self, scenario_definition: dict, driving_score: float
    ) -> dict:
        """
        Purely virtual method to be implemented by the user. Receives a
        scenario definition, results of a scenario run and the scenario critera.

        """
        raise NotImplementedError("This function should be implemented by the user")

    def _update_generator(self, seed: int) -> None:
        """Update the random seeded BitGenerator with a new seed"""
        self._rng = np.random.default_rng(seed)
