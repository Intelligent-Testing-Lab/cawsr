"""
This module provides a Autonomous agent to use with the official CARLA-Autoware bridge from 
the Autoware Universe. It uses ROS2.
"""

import math
import os
import threading
import signal
import time
import rclpy
import json

from dataclasses import dataclass

import numpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from autoware_vehicle_msgs.msg import Engage

from tier4_planning_msgs.msg import RouteState
from autoware_adapi_v1_msgs.msg import MotionState

from autoware_planning_msgs.msg import Trajectory
from autoware_internal_msgs.msg import MissionRemainingDistanceTime
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

#from srunner.autoagents.autonomous_agent import AutonomousAgent

from tf_transformations import quaternion_from_euler
#from builtin_interfaces.msg import Duration
# class AutowareAgent(AutonomousAgent):
class AutowareAgent():
    
    timestamp = None
    current_map_ = None
    published_plan = None
    _global_plan_world_coord = None
   
    def setup(self, path_to_conf_file: str | None = None):
        """Setup the Autoware Agent

        Args:
           path_to_conf_file (_str_): Config file 
        """
        
        if path_to_conf_file:
            # load the json coords
            with open(path_to_conf_file, 'r') as f:
                self._global_plan_world_coord = json.loads(f.read())

        self.autoware_state = AutowareState()
        self.node = RouteNode(self.autoware_state )
        self.logger = self.node.get_logger()
        
        self.current_map_ = 'Town01'
        
        self.waypoint_iteration = 0
        self.num_waypoints = len(self._global_plan_world_coord)
        self.finished_publishing = False
        
        if not self.finished_publishing:
            self.publish_points_dict()

    def publish_points(self) -> None:
        # check planning state = 2
        # publish goal pose
        # then checkpoints
        if self.autoware_state.no_route():
            goal_pose = self._global_plan_world_coord[-1]
            checkpoints = self._global_plan_world_coord[:-1]
            
            self.node.publish_goal(goal_pose)

            all_checkpoints = len(checkpoints) - 1
            i = 0

            while not self.finished_publishing:
                if all_checkpoints == i:
                    self.autoware_state.sent_route = True
                    self.finished_publishing = True


                if self.autoware_state.route_ready() or not self.autoware_state.planning_route():
                    self.node.publish_checkpoint(checkpoints[i])
                    i += 1


    def publish_points_dict(self) -> None:
        # check planning state = 2
        # publish goal pose
        # then checkpoints
        self.logger.info("Publishing Route...")
        if self.autoware_state.no_route():
            goal_pose = self._global_plan_world_coord[-1]
            checkpoints = self._global_plan_world_coord[:-1]

            pose = self.node.publish_goal_dict(goal_pose)
            self.node.publish_marker(pose)

            all_checkpoints = len(checkpoints) - 1 # 1
            i = 0 # 0 1

            while not self.finished_publishing:
                user_input = input("")
                if user_input.lower() == 'y':
                    if all_checkpoints == i:
                        self.autoware_state.sent_route = True
                        self.finished_publishing = True

                    if self.autoware_state.route_ready() or not self.autoware_state.planning_route():
                        checkpoint = self.node.publish_checkpoint_dict(checkpoints[i])
                        self.node.publish_marker(checkpoint)
                        i += 1
                    

        self.logger.info("Finished Publishing")

    def run_step(self):    
        #print(self.autoware_state.engage_ready())
        #print(self.autoware_state.sent_route)
        if self.autoware_state.engage_ready() and not self.autoware_state.sent_engage:
            self.node.send_engage(True)

        #if self.autoware_state.completed_route():
        #    self.autoware_state.reset_state()
    
        #if not (self.autoware_state.sent_route) and (self.autoware_state.within_goal()):
        #    # get waypoint
        #    waypoint = self._global_plan_world_coord[self.waypoint_iteration]
        #    self.node.publish_goal_dict(waypoint) # change to non dict in scenario runner 
        #    
        #    self.waypoint_iteration += 1
        #    
        #    self.logger.info("testing")
        #    
        #    # check last waypoint hasn't been published
        #    if self.waypoint_iteration > (self.num_waypoints - 1):
        #        self.waypoint_iteration = self.num_waypoints - 1
        #    
        #    self.logger.info("Setting route")
        #    self.logger.info(f"Set route: {waypoint}")
        #    self.autoware_state.sent_route = True
        #    
        #if self.autoware_state.engage_ready():
        #    self.node.send_engage(True)
        #    
        #if self.autoware_state.sent_engage:
        #    self.logger.info(f"DISTANCE TO GOAL: {self.autoware_state.distance_to_goal}")

        #    if (self.autoware_state.distance_to_goal < self.autoware_state.min_dist):
        #        self.node.send_engage(False)
        #
        #return
    
    def setup_sensors(self) -> None:
        return
   
@dataclass        
class AutowareState:
    """ Basic dataclass to encompass the state of Autoware 
    Used for deciding when to engage, send routes, etc...

    """
    position: Point = None

    sent_route: bool = False
    sent_engage: bool = False
    
    motion_state: int = 0
    planning_state: int = 0
    
    distance_to_goal: float = -1
    min_dist: float = 0.5
    zero_dist: float = -1 # distance when completed goal ?
        
    def within_goal(self) -> bool:
        return self.distance_to_goal >= self.min_dist or self.distance_to_goal == self.zero_dist
        
    def completed_route(self) -> bool:
        return self.planning_state == 6 or self.motion_state == 1
    
    def engage_ready(self) -> bool:
        return self.sent_route and self.planning_state == 4
    
    def no_route(self) -> bool:
        return self.planning_state == 2 or self.planning_state == 0
    
    def route_ready(self) -> bool:
        return self.planning_state == 4
   
    def planning_route(self) -> bool:
        return self.planning_state == 3

    def reset_state(self) -> None:
       self.sent_route = False
       self.sent_engage = False
       self.motion_state = 0
       self.planning_state = 0
       self.distance_to_goal = -1
   
class RouteNode(Node):
    
    route = '/planning/mission_planning/goal'
    engage = '/autoware/engage'
    route_state = '/planning/mission_planning/state'
    motion_state = '/api/motion/state'
    remaining_dist = '/planning/mission_remaining_distance_time'
    checkpoint = '/planning/mission_planning/checkpoint'
    marker_topic = '/visualization_marker'
    
    node_name = 'route_node'

    def __init__(self, autoware_state: AutowareState):
        super().__init__(self.node_name)
        
        self.logger = self.get_logger()
        self.state = autoware_state
        
        self.logger.info(f"Started Node {self.node_name}")
        
        self.route_pub_ = self.create_publisher(PoseStamped, self.route, 10)
        self.engage_pub_ = self.create_publisher(Engage, self.engage, 10)
        self.marker_pub_ = self.create_publisher(Marker, self.marker_topic, 10)
        self.checkpoint_pub_ = self.create_publisher(PoseStamped, self.checkpoint, 10)
        self.plan_state_sub_ = self.create_subscription(RouteState, self.route_state, self.plan_state_cb_, 10)
        self.motion_sub_ = self.create_subscription(MotionState, self.motion_state, self.motion_state_cb_ ,10)
        self.dist_sub_ = self.create_subscription(MissionRemainingDistanceTime, self.remaining_dist, self.remaining_dist_cb_, 10)
        
        self.logger.info(f"Started Publishers and Subscribers.")

    def publish_goal_dict(self, point) -> None:
        pose = PoseStamped()
        self.time_stamp = self.get_clock().now().to_msg()
        
        pose.header.frame_id = 'map'
        pose.header.stamp = self.time_stamp
        pose.pose.position.x = point['x']
        pose.pose.position.y = point['y']
        pose.pose.position.z = point['z']
        quaternion = quaternion_from_euler(
            0, 0, round(point['yaw']))
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.logger.info(f"[GOAL] Point: {point['x']} {point['y']} {point['z']} {quaternion}")

        self.route_pub_.publish(pose)
        return pose
            
    def publish_goal(self, point) -> None:
        pose = PoseStamped()
        self.time_stamp = self.get_clock().now().to_msg()
        
        pose.header.frame_id = 'map'
        pose.header.stamp = self.time_stamp
        pose.pose.position.x = point[0].location.x
        pose.pose.position.y = -point[0].location.y # may need to inverse, check performance first
        pose.pose.position.z = point[0].location.z
        quaternion = quaternion_from_euler(
            0, 0, -math.radians(point[0].rotation.yaw))
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.route_pub_.publish(pose)

    def publish_checkpoint(self, point) -> None:
        return
    
    def publish_checkpoint_dict(self, point) -> None:
        pose = PoseStamped()
        self.time_stamp = self.get_clock().now().to_msg()
        
        pose.header.frame_id = 'map'
        pose.header.stamp = self.time_stamp
        pose.pose.position.x = point['x']
        pose.pose.position.y = point['y']
        pose.pose.position.z = point['z']
        quaternion = quaternion_from_euler(
            0, 0, round(point['yaw'],1))
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.logger.info(f"[CHECKPOINT] Point: {point['x']} {point['y']} {point['z']} {quaternion}")

        self.checkpoint_pub_.publish(pose)
        return pose

    def send_engage(self, state: bool) -> None:
        engage_msg = Engage()
        engage_msg.engage = state
        self.engage_pub_.publish(engage_msg)
        self.get_logger().info(f"Engaging {state}")
        self.state.sent_engage = state


    def plan_state_cb_(self, route_msg: RouteState) -> None:
        """ Callback for ros2 topic
            /planning/mission_planning/state
        """
        curr = route_msg.state
        
        if not curr == self.state.planning_state:
            self.logger.info(f"Planning State: {curr}")
            self.state.planning_state = curr
            
    def traj_state_cb_(self, traj_msg: Trajectory) -> None:
        """ Callback for ros2 topic
            /planning/scenario_planning/trajectory'
        """
        self.state.position = traj_msg.points[-1].pose.position
    
    def motion_state_cb_(self, motion_msg: MotionState) -> None:
        """ Callback for ros2 topic
            /api/motion/state
        """
        curr = motion_msg.state
        
        if not curr == self.state.motion_state:
            self.logger.info(f"Motion State: {curr}")
            self.state.motion_state = curr
            
    def remaining_dist_cb_(self, time_distance_msg: MissionRemainingDistanceTime):
        self.state.distance_to_goal = time_distance_msg.remaining_distance


    def publish_marker(self, point) -> None:
        marker = Marker()

        scale_factor = 10.0
        marker.header.frame_id = point.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.ARROW
        marker.ns = 'goal_marker'
        marker.id = 0
        marker.action = marker.ADD
        marker.scale.x = 0.3 * scale_factor
        marker.scale.y = 0.05 * scale_factor
        marker.scale.z = 0.05 * scale_factor
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose = point.pose

        self.marker_pub_.publish(marker)
    
def tick_agent(agent, time_delay):
    while True:
        time.sleep(time_delay)
        agent.run_step()

def main(args=None):
    rclpy.init(args=args)
    agent = AutowareAgent()
    agent.setup('/scenario_runner/waypoints_valid.json')

    agent_thread = threading.Thread(target=tick_agent, args=(agent, 0.05,))
    agent_thread.start()
    
    rclpy.spin(agent.node)

    agent.node.destroy_node()
    agent_thread.join()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()