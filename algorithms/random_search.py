#!/bin/bash

# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from basic_algorithm import BasicAlgorithm

import lanelet2
import numpy as np


class RandomSearch(BasicAlgorithm):
    def __init__(self, args: dict) -> None:
        self._args = args

        self.lanelet2 = args["lanelet2"]
        self.prev_ds = 0
        self.all_points = self.__get_all_lanelet_points()  # stored in memory

    def _scenario_callback(
        self, scenario_definition: dict, driving_score: float
    ) -> dict:
        valid = False
        spawn = None
        goalpose = None

        while not valid:
            spawn = self._rng.choice(self.all_points)
            goalpose = self._rng.choice(self.all_points)

            valid = self.valid_route([spawn, goalpose])

        scenario_definition["routes"][0]["route"]["waypoints"] = [
            self._np_to_json(spawn),
            self._np_to_json(goalpose),
        ]
        return scenario_definition

    def _update_generator(self, seed: int) -> None:
        self._rng = np.random.default_rng(seed)

    def _np_to_json(self, p1: np.ndarray) -> dict:
        return {"position": {"x": p1[0], "y": p1[1], "z": 0.0}}

    def __get_all_lanelet_points(self) -> np.ndarray:
        laneletmap_ = lanelet2.io.load(self.lanelet2, lanelet2.io.Origin(0, 0))
        self.laneletmap_ = laneletmap_

        centerline_points = []
        for lanelet in list(self.laneletmap_.laneletLayer):
            for points in lanelet.centerline:
                centerline_points.append(np.asarray([points.x, points.y]))

        return np.asarray(centerline_points)  # convert to numpy array

    def _not_same_lane_check(self, p1, p2):
        p1 = lanelet2.core.BasicPoint2d(p1[0], p1[1])
        p2 = lanelet2.core.BasicPoint2d(p2[0], p2[1])

        lanelets = [
            lanelet2.geometry.findNearest(self.laneletmap_.laneletLayer, p1, 1),
            lanelet2.geometry.findNearest(self.laneletmap_.laneletLayer, p2, 1),
        ]

        lanelet_ids = [
            {ll.id for dist, ll in lanelets[0]},
            {ll.id for dist, ll in lanelets[1]},
        ]
        common_lanes = lanelet_ids[0].intersection(lanelet_ids[1])
        return not common_lanes

    def _dist(self, p1: np.ndarray, p2: np.ndarray) -> np.floating:
        return np.linalg.norm(p1 - p2)
