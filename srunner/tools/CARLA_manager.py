#!/bin/bash

# Copyright (c) 2025 University of Sheffield
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import subprocess
import os
import logging

from srunner.scenarioconfigs.carla_config import CARLA


logger = logging.getLogger("scenario-runner")
logger.propagate = False


class CARLAManager(object):
    container_id = None
    port = 2000  # default
    fidelity = "Low"  # default
    run_command = [
        'docker run -dt --gpus all --net=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=$DISPLAY -e NVIDIA_DRIVER_CAPABILITIES=all -e XDG_RUNTIME_DIR=/tmp carlasim/carla:0.9.15 /bin/bash -c "./CarlaUE4.sh -carla-rpc-port=2000 -quality-level=Low"'
    ]

    @staticmethod
    def _load_config(config: CARLA) -> None:
        CARLAManager.port = config.PORT
        CARLAManager.fidelity = config.FIDELITY

        CARLAManager.run_command = [
            f'docker run -dt --gpus all --net=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=$DISPLAY -e NVIDIA_DRIVER_CAPABILITIES=all -e XDG_RUNTIME_DIR=/tmp carlasim/carla:0.9.15 /bin/bash -c "./CarlaUE4.sh -carla-rpc-port={CARLAManager.port} -quality-level={CARLAManager.fidelity}"'
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
            CARLAManager.container_id = result.stdout.strip()
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
    def update_permission(path: str) -> None:
        """Updates the permissions of a folder to allow non-root users to copy

        Args:
            path (str): file to modify
        """
        env = os.environ.copy()
        result = subprocess.run(
            f"chmod -R 777 {path}",
            shell=True,
            text=True,
            capture_output=True,
            env=env,
        )

        if not result.returncode == 0:
            logger.error(f"Failed to update permissions for {path}")
        else:
            logger.info(f"Successfully changed {path} ownership")

    @staticmethod
    def fetch_file(path: str, dest: str):
        if CARLAManager.container_id is not None:
            env = os.environ.copy()
            # ensure everyone has permission to copy to dest
            result = subprocess.run(
                f"chmod -R 777 {dest} && docker cp {CARLAManager.container_id}:{path} {dest}",
                shell=True,
                text=True,
                capture_output=True,
                env=env,
            )
            logger.info(
                f"Copying... {path} from container {CARLAManager.container_id} to {dest}"
            )
            logger.info(f"docker cp {CARLAManager.container_id}:{path} {dest}")

            if not result.returncode == 0:
                logger.info(f"Failed to copy {path} to {dest}")

    @staticmethod
    def restart_carla():
        if CARLAManager.container_id is not None:
            env = os.environ.copy()
            result = subprocess.run(
                f"docker container restart {CARLAManager.container_id}",
                shell=True,
                text=True,
                capture_output=True,
                env=env,
            )
            logger.info(
                f"Restarted CARLA container {CARLAManager.container_id} with exit code {result.returncode}"
            )
            CARLAManager.container_id = result.stdout.strip()
        else:
            CARLAManager.start_carla()
