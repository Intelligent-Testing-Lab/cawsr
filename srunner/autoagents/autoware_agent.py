from srunner.autoagents.autonomous_agent import AutonomousAgent

from srunner.autoagents.autoware_nodes.autoware_types import waypoint
from srunner.autoagents.autoware_nodes import autoware_node
from srunner.autoagents.autoware_nodes import route_node
from srunner.autoagents.autoware_nodes import state_node

from srunner.autoagents.agent_state import autoware_state

from srunner.scenarioconfigs.environment_configuration import EnvironmentConfig

from autoware_carla_interface_msgs.msg import EgoConfig, SensorConfig

import threading
import rclpy
import time
import logging

logger = logging.getLogger("scenario-runner")
logger.propagate = False


class AutowareAgent(AutonomousAgent):
    timestamp = None
    agent_set_route = False
    counter = 0

    def setup(self, config: EnvironmentConfig | None = None) -> None:
        """Setup the Autoware Agent.
            - Initialise the state
            - Setup nodes

        Args:
            config (EnvironmentConfig | None): environment configuration file
        """

        rclpy.init(args=None)
        self.config = config

        self.autoware_state = autoware_state.AutowareState("ego_vehicle", None)

        self.state_node = state_node.StateNode(self.autoware_state)
        self.state_node.reset_autoware()

        self.route_node = route_node.RouteNode(self.autoware_state)
        self.autoware_node = autoware_node.AutowareNode(self.autoware_state)

        self._multi_thread_executor = rclpy.executors.MultiThreadedExecutor()

        self._multi_thread_executor.add_node(self.route_node)
        self._multi_thread_executor.add_node(self.state_node)
        self._multi_thread_executor.add_node(self.autoware_node)

        self._executor_thread = threading.Thread(
            target=self._multi_thread_executor.spin, daemon=True
        )
        self._executor_thread.start()

        self.sent_route = False

        self.publish_sensor_state()

    def publish_sensor_state(self) -> None:
        # publish sensor information to the bridge
        # wait for it to return the correct message
        ego_config_msg = EgoConfig()
        ego_config_msg.ego_name = self.config.ego_name  # type: ignore
        ego_config_msg.ego_model = self.config.ego_model  # type: ignore
        ego_config_msg.sensors = []

        for sensor_config in self.config.sensor_config:  # type: ignore
            sensor_config_msg = SensorConfig()
            sensor_config_msg.sensor_type = sensor_config.type
            sensor_config_msg.sensor_id = sensor_config.id
            ego_config_msg.sensors.append(sensor_config_msg)

        # keep publishing ego_sensor config until the bridge is ready
        # big performance diminishment here
        while not self.autoware_state.bridge_ready:
            logger.info("Sending Sensor state to Agent...")
            time.sleep(5) # DO NOT CHANGE THIS IS A MAGIC NUMBER
            self.state_node.ego_config_publisher.publish(ego_config_msg)

    def set_route(self) -> None:
        self.agent_set_route = True

        self.goal_pose_world = self._global_plan_world_coord[-1]
        self.waypoints_world = self._global_plan_world_coord[:-1]

        logger.info("Clearing route...")
        self.route_node.request_clear_route()

    def _convert_to_waypoint(self, point):
        """Returns a waypoint

        Args:
            point (Point): Point to convert
        """
        return waypoint.Waypoint(
            point[0].location.x,
            point[0].location.y,
            point[0].location.z,
            node=self.autoware_node,
        )

    def cleanup(self) -> None:
        """Cleanup"""
        logger.info("Sending shutdown signal to autoware...")
        self.state_node.reset_autoware()
        logger.info("Waiting for shutdown. Starting Node cleanup")
        time.sleep(1)  # sleep for 1 second for sanity
        try:
            self.autoware_node.destroy_node()
            self.state_node.destroy_node()
            self.route_node.destroy_node()
            rclpy.shutdown()
            self._executor_thread.join()
        except RuntimeError:
            logger.info("Failed to clean up executor thread...")

    def run_step(self) -> None:
        """Tick method containing all logic based on autoware state"""
        self.counter += 1
        if self.counter % 20 == 0:
            logger.info("Ticked 1 second")
        
        if not self.agent_set_route:
            self.set_route()

        if (
            self.autoware_state.is_ready_publish_route()
            and self.agent_set_route
            and not self.sent_route
        ):
            waypoints = []
            goal_pose = self._convert_to_waypoint(
                self.goal_pose_world
            ).autoware_from_world_coords()

            for waypoint in self.waypoints_world:
                waypoints.append(
                    self._convert_to_waypoint(waypoint).autoware_from_world_coords()
                )

            # autoware cannot handle many waypoints, becomes unreliable
            n_waypoints = len(waypoints)
            segment_size = int(n_waypoints / 3)

            # self.route_node.request_route(goal_pose, waypoints[0::segment_size])
            self.route_node.publish_route(goal_pose, waypoints[0::segment_size])
            self.sent_route = True

        # check if the current route is set
        if self.autoware_state.route_set() and not self.autoware_state.sent_engage:
            self.autoware_node.publish_engage(True)
