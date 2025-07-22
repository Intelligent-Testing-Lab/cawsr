from srunner.autoagents.autonomous_agent import AutonomousAgent

from srunner.autoagents.autoware_nodes.autoware_types import waypoint
from srunner.autoagents.autoware_nodes import autoware_node
from srunner.autoagents.autoware_nodes import route_node
from srunner.autoagents.autoware_nodes import state_node

from srunner.autoagents.agent_state import autoware_state

import threading
import rclpy


DEBUG_ENV = False

# uncomment if testing in scenario runner


class AutowareAgent(AutonomousAgent):
    timestamp = None
    current_map = None
    agent_set_route = False

    def setup(self, path_to_conf_file: dict | None = None) -> None:
        """Setup the Autoware Agent.
            - Initialise the state
            - Setup nodes

        Args:
            _path_to_conf (dict | None): path to config, passed from AutonomousAgent
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

    def set_route(self) -> None:
        # for every point in the plan
        # convert to waypoint
        # get the autoware pose
        # publish

        self.goal_pose_world = self._global_plan_world_coord[-1]
        self.waypoints_world = self._global_plan_world_coord[:-1]

        # reinitialise localization
        # self.autoware_node.request_localize()  # None uses GNSS

        # clear route
        # self.route_node.request_clear_route()

        self.agent_set_route = True

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
        if not self.agent_set_route:
            self.set_route()

        if self.autoware_state.is_ready_publish_route():
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
