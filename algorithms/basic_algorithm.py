class BasicAlgorithm(object):
    def __init__(self, args: dict) -> None:
        self._args = args

    def _scenario_callback(
        self, scenario_definition: dict, driving_score: float
    ) -> dict:
        """
        Purely virtual method to be implemented by the user. Receives a
        scenario definition, results of a scenario run and the scenario critera.

        """
        raise NotImplementedError("This function should be implemented by the user")
