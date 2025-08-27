import subprocess
import time

class CARLAManager(object):
    container_id = None
    run_command = (
        "docker run -d --rm"
        "--gpus all"
        "-e NVIDIA_VISIBLE_DEVICES=all"
        "-e DISPLAY=:0"
        "-v /tmp/.X11-unix:/tmp/.X11-unix:rw"
        "-e XDG_RUNTIME_DIR=/tmp"
        "--network host"
        "carlasim/carla:0.9.15"
        "/bin/bash -c '/home/carla/CarlaUE4.sh'"
    )
    
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
        
        
def main():
    CARLAManager.start_carla()
    time.sleep(15)
    CARLAManager.stop_carla()
    
if __name__ == "__main__":
    main()