import subprocess
import threading

class CARLAManager(object):
    container_id = None
    run_command = """
        docker run -d --rm \
    --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e SQL_LVDELDRIVER=x11 \
    -e DISPLAY=:1 \
    -e XDG_RUNTIME_DIR=/tmp \
    -e NVIDIA_REQUIRE_CUDA="cuda>=10.1 brand=tesla,driver>=384 driver>=384 brand=tesla,driver>=396 driver>=396 brand=tesla,driver>=410 driver>=410" \
    --network host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    carlasim/carla:0.9.15 \
    /bin/bash -c "/home/carla/CarlaUE4.sh"
    """
    
    @staticmethod
    def start_carla():
        result = subprocess.run(
            CARLAManager.run_command,
            shell=True,
            text=True,
            capture_output=True
        )
        CARLAManager.container_id = result.stdout.strip()
        
    @staticmethod    
    def stop_carla():
        if CARLAManager.container_id is not None:
            subprocess.run(
                f"docker container stop {CARLAManager.container_id}",
                shell=True
            )
            CARLAManager.container_id = None
        
    @staticmethod
    def restart_carla():
        CARLAManager.stop_carla()
        CARLAManager.start_carla()