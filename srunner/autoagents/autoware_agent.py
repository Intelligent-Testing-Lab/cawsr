from srunner.autoagents.autonomous_agent import AutonomousAgent

from srunner.autoagents.autoware_nodes.autoware_types import waypoint
from srunner.autoagents.autoware_nodes import autoware_node
from srunner.autoagents.autoware_nodes import route_node
from srunner.autoagents.autoware_nodes import state_node

from srunner.autoagents.agent_state import autoware_state

from srunner.tools.environment_parser import EnvironmentConfig

from autoware_carla_interface.msg import EgoConfig, SensorConfig

import threading
import rclpy
import time


DEBUG_ENV = False


# uncomment if testing in scenario runner
class AutowareAgent(AutonomousAgent):
    timestamp = None
    current_map = None
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

        self.autoware_state = autoware_state.AutowareState("ego_vehicle", None)

        self.route_node = route_node.RouteNode()
        self.state_node = state_node.StateNode(self.autoware_state)
        self.autoware_node = autoware_node.AutowareNode(self.autoware_state)

        self._nodes = [self.route_node, self.autoware_node, self.state_node]
        self._node_threads = [
            threading.Thread(target=rclpy.spin, args=(self.route_node)),
            threading.Thread(target=rclpy.spin, args=(self.autoware_node)),
            threading.Thread(target=rclpy.spin, args=(self.state_node)),
        ]

        # check the bridge is ready
        # publish sensor information to the bridge
        # wait for it to return the correct message
        # hang until

        ego_config_msg = EgoConfig()
        ego_config_msg.ego_name = config.ego_name
        ego_config_msg.ego_model = config.ego_model
        ego_config_msg.sensors = []

        for sensor_config in config.sensor_config:
            sensor_config_msg = SensorConfig()
            sensor_config_msg.sensor_type = sensor_config.type
            sensor_config_msg.sensor_id = sensor_config.id
            ego_config_msg.sensors.append(sensor_config_msg)

        # keep publishing ego_sensor config until the bridge is ready
        while not self.autoware_state.bridge_ready:
            time.sleep(1)
            self.autoware_state.ego_config_publisher.publish(ego_config_msg)

    def set_route(self) -> None:
        # for every point in the plan
        # convert to waypoint
        # get the autoware pose
        # publish
        self.agent_set_route = True

        self.goal_pose_world = self._global_plan_world_coord[-1]
        self.waypoints_world = self._global_plan_world_coord[:-1]

        # reinitialise localization
        print("called localise")
        # self.autoware_node.request_localize()  # None uses GNSS

        print("called clear route")
        # clear route
        # self.route_node.request_clear_route()

    def _convert_to_waypoint(self, point):
        """Returns a waypoint

        Args:
            point (Point): Point to convert
        """
        return waypoint.Waypoint(
            point[0].location.x,
            point[0].location.y,
            point[0].location.z,
            point[0].rotation.yaw,
        )

    def destroy(self) -> None:
        """Cleanup"""
        try:
            for thread in range(len(self._node_threads)):
                self._node_threads[thread].join()
                self._nodes[thread].destroy_node()
        except RuntimeError:
            print("Cleaned up threads...")

        rclpy.shutdown()

    def run_step(self) -> None:
        """Tick method containing all logic based on autoware state"""
        self.counter += 1
        if self.counter % 20 == 0:
            print("1 second")

        if not self.agent_set_route:
            self.set_route()

        if self.autoware_state.is_ready_publish_route() and self.agent_set_route:
            waypoints = []
            goal_pose = self._convert_to_waypoint(
                self.goal_pose_world
            ).autoware_from_world_coords()
            for waypoint in self.waypoints_world:
                waypoints.append(
                    self._convert_to_waypoint(waypoint).autoware_from_world_coords()
                )
            self.route_node.publish_route(goal_pose, waypoints)
            self.autoware_state.sent_route = True

        # check if the current route is set
        if self.autoware_state.route_ready() and not self.autoware_state.sent_engage:
            self.autoware_node.publish_engage(True)
