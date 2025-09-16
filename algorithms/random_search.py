from basic_algorithm import BasicAlgorithm
from srunner.tools import route_manipulation

import lanelet2
import carla
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

            valid = self._valid_route([spawn, goalpose]) and self._not_same_lane_check(
                spawn, goalpose
            )

        scenario_definition["routes"][0]["route"]["waypoints"] = [
            self._np_to_json(spawn),
            self._np_to_json(goalpose),
        ]
        return scenario_definition

    def _update_generator(self, seed: int) -> None:
        self._rng = np.random.default_rng(seed)

    def _to_carla(self, point: np.ndarray) -> carla.Location:
        return carla.Location(point[0], point[1], 0.0)

    def _valid_route(self, route) -> bool:
        carla_route = list(map(self._to_carla, route))

        gps_route, route = route_manipulation.interpolate_trajectory(carla_route)
        return not ((len(gps_route) == 1) and (len(route) == 1))

    def _np_to_json(self, p1: np.ndarray) -> dict:
        return {"position": {"x": p1[0], "y": p1[1], "z": 0.0}}

    def __get_all_lanelet_points(self) -> np.ndarray:
        map = lanelet2.io.load(self.lanelet2, lanelet2.io.Origin(0, 0))
        lanelets = map.laneletLayer
        self.lanelet_map = lanelets

        centerline_points = []
        for lanelet in list(lanelets):
            for points in lanelet.centerline:
                centerline_points.append(np.asarray([points.x, points.y]))

        return np.asarray(centerline_points)  # convert to numpy array

    def _not_same_lane_check(self, p1, p2):
        lanelets = [
            lanelet2.geometry.findWithin(self.lanelet_map, p1, 0),
            lanelet2.geometry.findWithin(self.lanelet_map, p2, 0),
        ]

        lanelet_ids = [
            {ll.id for dist, ll in lanelets[0]},
            {ll.id for dist, ll in lanelets[1]},
        ]
        common_lanes = lanelet_ids[0].intersection(lanelet_ids[1])
        return common_lanes == 0  # 0 means no shared lanes

    def _dist(self, p1: np.ndarray, p2: np.ndarray) -> np.floating:
        return np.linalg.norm(p1 - p2)
