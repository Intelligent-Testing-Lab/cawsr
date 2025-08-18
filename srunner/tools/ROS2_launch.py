import subprocess
import threading


class ROS2Launch(object):
    threads = dict()
    stop_event = threading.Event()

    @staticmethod
    def launch_file(
        package_name: str, launch_file: str, args: dict | None = None
    ) -> None:
        """will launch the file specified from the package specified

        Args:
            package_name (str): ROS2 package name
            launch_file (str): launch file name
            args (dict): dictionary of arguments to pass to launch file
        """
        command = ROS2Launch._construct_command(package_name, launch_file, args)
        thread = threading.Thread(
            target=ROS2Launch._run_command, args=(command, ROS2Launch.stop_event)
        )
        ROS2Launch.threads[package_name] = {"thread": thread, "stop": False}
        thread.start()

    @staticmethod
    def cleanup(package_name: str) -> None:
        """provide package name to stop running

        Args:
            package_name (str): name of the ROS2 package to kill
        """
        if package_name in ROS2Launch.threads:
            thread = ROS2Launch.threads[package_name]
            thread["stop"] = True
            thread["thread"].join()

    @staticmethod
    def _run_command(command: list[str], package_name) -> None:
        process = subprocess.Popen(command, check=True)

        if package_name in ROS2Launch.threads:
            while not ROS2Launch.threads[package_name]["stop"]:
                if process.poll() is not None:
                    break

            if process.poll() is None:
                process.kill()

    @staticmethod
    def _construct_command(
        package_name: str, launch_file: str, args: dict | None = None
    ) -> list[str]:
        command = []
        command.append("ros2 launch").append(package_name).append(launch_file)

        if args is not None:
            for key, value in args.dict():
                command.append(f"{key}:={value}")
        return command
