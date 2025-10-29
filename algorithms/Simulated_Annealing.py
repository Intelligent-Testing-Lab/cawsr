#!/bin/bash

# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from BasicAlgorithm import BasicAlgorithm
import lanelet2
import random
import math

from math import sqrt, pow


class Simulated_Annealing(BasicAlgorithm):
    def __init__(self, args: dict) -> None:
        super(BasicAlgorithm).__init__(args)
        self.temperature = args["temperature"]
        self.min_temperature = args["min_temperature"]
        self.temperature_step = args["temperature_step"]

        self.radius = args["radius"]
        self.lanelet_path = args["lanelet_path"]

        self.prev_ds = None
        self.prev_waypoints = None
        self.waypoints_index = 0

    def _scenario_callback(
        self, scenario_definition: dict, driving_score: float
    ) -> dict:
        waypoints = scenario_definition["routes"][0]["waypoints"]

        if self.temperature <= self.min_temperature:
            print("Reached min temperature, no more iterations needed")
            print("=========final waypoint values are=========")
            print(self.prev_waypoints)
            print("=========final driving score===============")
            print(self.prev_ds)
            print("===========================================")
            # solution is prev_ds and prev_waypoints
            return

        if self.prev_ds is not None:
            ds_diff = self.prev_ds - driving_score  # we want a worse driving score
            ds_diff = driving_score - self.prev_ds
            if ds_diff > 0:
                self.prev_waypoints = waypoints[self.waypoint_index]
                self.prev_ds = driving_score
            elif math.exp(ds_diff / self.temperature) > random.randint(0, 1):
                self.prev_waypoints = waypoints[self.waypoint_index]
                self.prev_ds = driving_score

            self.temperature -= self.temperature_step
        else:
            self.prev_ds = driving_score
            self.prev_waypoints = waypoints[self.waypoint_index]

        new_point = self.__find_new_neighbour_point(waypoints[self.waypoints_index])
        waypoints[self.waypoints_index] = new_point

        if self.waypoint_index + 1 > len(waypoints):
            self.waypoint_index = 0
        else:
            self.waypoint_index += 1

        return scenario_definition

    def __find_new_neighbour_point(self, current_point: dict) -> dict:
        current_point_ = (
            current_point["position"]["x"],
            current_point["position"]["y"],
        )

        all_points = self.__get_all_lanelet_points()
        self.visited_points.add(current_point_)
        all_points.difference(self.visited_points)

        points_in_radius = []
        for point in all_points:
            if (
                self.radius == -1
                or self.__euclidian_distance(point, current_point_) <= self.radius
            ):
                points_in_radius.append(point)

        if len(points_in_radius) == 0:
            random_point = random.choice(list(all_points))
        else:
            random_point = random.choice(points_in_radius)

        return {"positions": {"x": random_point[0], "y": random_point[1]}}

    def __get_all_lanelet_points(self) -> set[tuple[int, int]]:
        map = lanelet2.io.load(self.lanelet_path, lanelet2.io.Origin(0, 0))
        lanelets = map.laneletLayer

        centerline_points = []
        for lanelet in list(lanelets):
            for points in lanelet.centerline:
                centerline_points += (points.x, points.y)

        return set(centerline_points)

    def __euclidian_distance(self, point1, point2) -> float:
        return sqrt(pow(point1.x - point2.x, 2) + pow(point2.y - point1.x, 2))
