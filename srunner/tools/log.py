from pathlib import Path
from shutil import rmtree
from datetime import datetime
import os
from typing import Union

class LogUtil():
    
    @staticmethod
    def clear_logs(logger: str = "") -> None:
        """
        Deletes all log files. If ```logger``` is specified, only files associated with the ```logger``` will be deleted
       
        Args:
            logger? (str): name of logger 
    
        Returns: 
            None
        """
        if logger:
            logs_path = Path('logs/', logger)
            LogUtil._clear_folder(logs_path, '.log')
            return
        
        logs_path = Path('logs/').absolute()

        # clear all logs 
        [LogUtil._clear_folder(os.path.join(logs_path, log), '.log') for log in os.listdir(logs_path) if os.path.isdir(os.path.join(logs_path, log))]

    @staticmethod
    def _clear_folder(path: Union[str, Path], extension: str) -> None:
        """
        Deletes all files in a specified directory.
        
        Args:
            path (str): path to directory
            extension (str): only files with the extension will be deleted
        
        Returns:   
            None
        """
        # get absolute path of folder
        if type(path) is str:
            path = Path(path)
        
        # get all files with extension
        files = [x for x in os.listdir(path) if os.path.isfile(os.path.join(path, x)) and os.path.splitext(x)[1] == extension]
        
        for log_file in files:
            os.remove(os.path.join(path, log_file))
        
    @staticmethod
    def create_log_file(path: str) -> Path:
        """
            Returns a full path to log file with the current datetime as the filename
        
        Args:
            path (str): relative path to the log folder
        
        Returns:
            str: full path to log file 
        """
        log = Path("{path}{filename}.log".format(
            path=path, 
            filename=datetime.now().strftime('%d.%m.%Y - %H.%M.%S'))).absolute()

        # create new log file        
        with open(log, 'w') as fp:
            fp.close()
 
        return log 
    
        
        
        
        
        
    
    