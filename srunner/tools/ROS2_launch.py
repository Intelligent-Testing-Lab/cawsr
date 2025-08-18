import subprocess
import threading

class ROS2Launch(object):
    threads = dict()
    stop_event = threading.Event()
        
    @classmethod
    def launch_file(self, package_name: str, launch_file: str, args: dict) -> None:
        """will launch the file specified from the package specified

        Args:
            package_name (str): ROS2 package name
            launch_file (str): launch file name
            args (dict): dictionary of arguments to pass to launch file
        """
        command = self._construct_command(package_name, launch_file, args)
        thread = threading.Thread(target=self._run_command, args=(command, self.stop_event))
        self.threads[self.package_name] = {
            "thread": thread,
            "stop": False
        }
        thread.start()
    
    @classmethod
    def cleanup(self, package_name: str) -> None:
        """provide package name to stop running

        Args:
            package_name (str): name of the ROS2 package to kill
        """
        if package_name in self.threads:
            thread = self.threads[package_name]
            thread['stop'] = True
        
    def _run_command(self, command: list[str], package_name) -> None:
        process = subprocess.Popen(command, check=True)
        
        if package_name in self.threads:
            while(not self.threads[package_name]['stop']):
                if process.poll() is not None:
                    break
                
            if process.poll() is None:
                process.kill()
        
    def _construct_command(self, package_name, launch_file, args) -> list[str]:
        command = []
        command.append("ros2 launch").append(package_name).append(launch_file)
        for key, value in args.dict():
            command.append(f"{key}:={value}")
        return command
            
        