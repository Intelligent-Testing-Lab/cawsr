import os
import datetime


class ResultsManager(object):
    def __init__(self) -> None:
        return

    def _create_run_folder(self, base_path: str = "results"):
        now = datetime.datetime.now()
        full_path = os.path.join(base_path, f"run-{now.strftime('%Y-%m-%d-%H-%M-%S')}")  # type: ignore

        try:
            os.makedirs(full_path, exist_ok=True)  # stops exceptions
            return full_path
        except OSError as e:
            print(f"Error: Could not create directory '{full_path}'.")
            print(f"Reason: {e}")

    def _create_scenario_folder(
        self, scenario: str, iteration: int, results_folder: str
    ):
        full_path = os.path.join(results_folder, f"{scenario}-0{iteration}")

        try:
            os.makedirs(full_path, exist_ok=True)
            return full_path
        except OSError as e:
            print(f"Error: Could not create directory '{full_path}'.")
            print(f"Reason: {e}")
