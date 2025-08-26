class BasicAlgorithm(object):
    def __init__(self, args: dict) -> None:
        self._args = args

    def _scenario_callback(
        self, scenario_definition: dict, driving_score: float
    ) -> dict:
        return scenario_definition
