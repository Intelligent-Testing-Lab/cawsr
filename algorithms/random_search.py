from basic_algorithm import BasicAlgorithm
import lanelet2

import numpy as np


class RandomSearch(BasicAlgorithm):
    def __init__(self, args: dict) -> None:
        self._args = args

        self.lanelet2 = args["lanelet2"]
        self.seed = args["seed"]

        self.bounds = [args["lower_bound"], args["upper_bound"]]
        # initialise the numpy seeded generator
        self._rng = np.random.default_rng(self.seed)

        # state
        self.prev_ds = 0
        self.all_points = self.__get_all_lanelet_points()  # stored in memory

    def _scenario_callback(
        self, scenario_definition: dict, driving_score: float
    ) -> dict:
        valid = False
        spawn = None
        checkpoint = None

        while not valid:
            spawn = self._rng.choice(self.all_points)
            checkpoint = self._rng.choice(self.all_points)

            if self.bounds[0] < self._dist(spawn, checkpoint) < self.bounds[1]:
                valid = True

        scenario_definition["routes"][0]["route"]["waypoints"] = [spawn, checkpoint]
        return scenario_definition

    def _np_to_json(self, p1: np.ndarray) -> dict:
        return {"position": {"x": p1[0], "y": p1[1], "z": 0.0}}

    def __get_all_lanelet_points(self) -> np.ndarray:
        map = lanelet2.io.load(self.lanelet2, lanelet2.io.Origin(0, 0))
        lanelets = map.laneletLayer

        centerline_points = []
        for lanelet in list(lanelets):
            for points in lanelet.centerline:
                centerline_points.append(np.asarray([points.x, points.y]))

        return np.asarray(centerline_points)  # convert to numpy array

    def _dist(self, p1: np.ndarray, p2: np.ndarray) -> np.floating:
        return np.linalg.norm(p1 - p2)
