class BasicAlgorithm(object):
    def __init__(self) -> None:
        return

    def _scenario_callback(
        self, scenario_definition: dict, scenario_critera: dict, results: dict
    ) -> dict:
        """
        Purely virtual method to be implemented by the user. Receives a
        scenario definition, results of a scenario run and the scenario critera.

        """
        raise NotImplementedError("This function should be implemented by the user")
