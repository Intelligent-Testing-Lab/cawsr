import os
import datetime


class ResultsManager(object):
    """Class to manage the file structure of the results. Creates folders for entire experiments (runs) and individual scenario executions (scenarios)
    Holds the file path of the last scenario executes, and the results.
    
    """
    
    def __init__(self, output_dir: str = 'results') -> None:
        self.output_dir = output_dir
        self.last_scenario = ''
        self.results_path = ''
        
    def create_run_folder(self, base_path: str = ''):
        """Creates a experiment folder under base_path. The naming convention is as follows
        ```
        run-{yy-mm-dd-h-m-s}
        ```

        Args:
            base_path (str, optional): Root folder. Defaults to 'results'.

        Returns:
            _type_: None
        """
        if not base_path:
            base_path = self.output_dir
        
        now = datetime.datetime.now()
        full_path = os.path.join(base_path, f"run-{now.strftime('%Y-%m-%d-%H-%M-%S')}")  # type: ignore

        os.makedirs(full_path, exist_ok=True)  # exist_ok=True, no need to error handle
        self.results_path = full_path
        return full_path

    def create_scenario_folder(
        self, scenario: str, iteration: str, results_folder: str
    ):
        """Creates a folder to hold the results of a individual scenario execution.
        The naming convention is as follows
        ```
        {results_folder}/{scenario}-{iteration}
        ```

        Args:
            scenario (str): Name of the scenario executed
            iteration (str): ID of the scenario (can by anything, but must be unique)
            results_folder (str): Root folder

        Returns:
            _type_: _description_
        """
        
        full_path = os.path.join(results_folder, f"{scenario}-{iteration}")

        os.makedirs(full_path, exist_ok=True) # exist_ok=True, no need to error handle
        self.last_scenario = full_path
        return full_path
