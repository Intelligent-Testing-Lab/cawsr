import subprocess
import os
import time
import logging

logger = logging.getLogger("scenario-runner")
logger.propagate = False


class CARLAManager(object):
    container_id = None
    run_command = [
        'docker run -dt --gpus all --net=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=$DISPLAY -e NVIDIA_DRIVER_CAPABILITIES=all -e XDG_RUNTIME_DIR=/tmp carlasim/carla:0.9.15 /bin/bash -c "./CarlaUE4.sh -quality-level=Low"'
    ]

    @staticmethod
    def start_carla():
        # need a way to verify CARLA has launched
        if CARLAManager.container_id is None:
            env = os.environ.copy()
            result = subprocess.run(
                CARLAManager.run_command,
                shell=True,
                text=True,
                capture_output=True,
                env=env,
            )
            logger.info(f"Started CARLA container {result.stdout.strip()}")
        else:
            CARLAManager.restart_carla()

    @staticmethod
    def stop_carla():
        if CARLAManager.container_id is not None:
            env = os.environ.copy()
            result = subprocess.run(
                f"docker container kill {CARLAManager.container_id}",
                shell=True,
                text=True,
                capture_output=True,
                env=env,
            )
            logger.info(
                f"Killed CARLA container {CARLAManager.container_id} with exit code {result.returncode}"
            )
            logger.info(f"Kill stdout: {result.stdout.strip()}")
            CARLAManager.container_id = None

    @staticmethod
    def restart_carla():
        CARLAManager.stop_carla()
        time.sleep(5)
        CARLAManager.start_carla()
