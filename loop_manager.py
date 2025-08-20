import yaml
import aw_scenario_runner
import rclpy

from std_msgs.msg import Empty
from srunner.tools.ROS2_launch import ROS2Launch


def main():
    AW_RESTART_TOPIC = "autoware/restart"

    node = rclpy.create_node("loop_manager")
    restart_aw_publisher = node.create_publisher(Empty, AW_RESTART_TOPIC, 10)

    config = None
    with open("config.yaml", "r") as stream:
        config = yaml.safe_load(stream)

    ITERATIONS = config["scenario_runner"]["algorithms"]["iterations"]

    for _ in range(0, ITERATIONS):
        
        # restart / start autoware
        restart_aw_publisher.publish(Empty())
        
        # launch bridge
        ROS2Launch.launch_file(
            "autoware_carla_interface", "autoware_carla_interface", dict()
        )

        # launch scenario runner
        aw_scenario_runner.main()


if __name__ == "__main__":
    main()
