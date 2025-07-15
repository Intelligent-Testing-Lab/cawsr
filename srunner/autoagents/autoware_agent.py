"""
This module provides a Autonomous agent to use with the official CARLA-Autoware bridge from 
the Autoware Universe. It uses ROS2.
"""

import math
import os
import threading
import signal
import time

from dataclasses import dataclass

import numpy

import rclpy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from autoware_vehicle_msgs.msg import Engage
from tier4_planning_msgs.msg import RouteState
from autoware_adapi_v1_msgs.msg import MotionState
from autoware_planning_msgs.msg import Trajectory
from autoware_internal_msgs.msg import MissionRemainingDistanceTime
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from tf_transformations import quaternion_from_euler

#from srunner.autoagents.autonomous_agent import AutonomousAgent
# class AutowareAgent(AutonomousAgent):
class AutowareAgent():
    
    timestamp = None
    current_map_ = None
    published_plan = None
    _global_plan_world_coord = None
   
    def setup(self, path_to_conf_file: dict):
        """Setup the Autoware Agent

        Args:
           path_to_conf_file (_dict_): Config file 
        """
        
        self.node = RouteNode()
        self.logger = self.node.get_logger()
        
        self.autoware_state = AutowareState()
        self.current_map_ = 'Town01'
        
        self.waypoint_iteration = 0
        self.num_waypoints = len(self._global_plan_world_coord)
    
    def run_step(self):
        if self.autoware_state.completed_route():
            self.autoware_state.reset_state()
        
        if not (self.autoware_state.sent_route) and (self.autoware_state.within_goal()):
            # get waypoint
            waypoint = self._global_plan_world_coord[self.waypoint_iteration]
            self.node.publish_goal(waypoint)
            
            self.waypoint_iteration += 1
            
            # check last waypoint hasn't been published
            if self.waypoint_iteration > (self.num_waypoints - 1):
                self.waypoint_iteration = self.num_waypoints - 1
            
            self.logger.info("Setting route")
            self.logger.info(f"Set route: {waypoint}")
            self.autoware_state.sent_route = True
            
        if self.autoware_state.engage_ready():
            self.node.send_engage(True)
            
        if self.autoware_state.sent_engage:
            self.logger.info(f"DISTANCE TO GOAL: {self.autoware_state.distance_to_goal}")

            if (self.autoware_state.distance_to_goal < self.autoware_state.min_dist):
                self.node.send_engage(False)
        
        return
    
    def setup_sensors(self) -> None:
        return
   
@dataclass        
class AutowareState:
    """ Basic dataclass to encompass the state of Autoware 
    Used for deciding when to engage, send routes, etc...

    """
    sent_route: bool = False
    sent_engage: bool = False
    
    motion_state: int = 0
    planning_state: int = 0
    
    distance_to_goal: float = 0
    min_dist: float = 0.5
    zero_dist: float = -1 # distance when completed goal ?
    position: Point
        
    def within_goal(self):
        return True if (self.distance_to_goal >= self.min_dist or self.distance_to_goal == self.zero_dist) else False
        
    def completed_route(self) -> bool:
        return True if (self.planning_state == 6 or self.motion_state == 1) else False
    
    def engage_ready(self) -> bool:
        return True if (self.sent_route and self.planning_state == 4) else False
   
    def reset_state(self) -> None:
       self.sent_route = False
       self.sent_engage = False
       self.distance_to_goal = -1
   
class RouteNode(rclpy.Node):
    
    route = '/planning/mission_planning/goal'
    engage = '/autoware/engage'
    marker = '/visualization_marker'
    route_state = '/planning/mission_planning/state'
    trajectory = '/planning/scenario_planning/trajectory'
    motion_state = '/api/motion/state'
    remaining_dist = '/planning/mission_remaining_distance_time'
    
    send_marker = False
    node_name = 'route_node'


    def __init__(self, autoware_state: AutowareState):
        super().__init__(self.node_name)
        
        self.logger = self.get_logger()
        self.state = autoware_state
        
        self.logger.info(f"Started Node {self.node_name}")
        
        self.route_pub_ = self.create_publisher(PoseStamped, self.route, 10)
        self.engage_pub_ = self.create_publisher(Engage, self.engage, 10)
        self.plan_state_sub_ = self.create_subscription(RouteState, self.route_state, self.plan_state_cb_, 10)
        self.trajectory_sub_ = self.create_subscription(Trajectory, self.trajectory, self.traj_state_cb_, 10)
        self.motion_sub_ = self.create_subscription(MotionState, self.motion_state, self.motion_state_cb_ ,10)
        self.dist_sub_ = self.create_subscription(MissionRemainingDistanceTime, self.remaining_dist, self.remaining_dist_cb_, 10)
        
        self.logger.info(f"Started Publishers and Subscribers.")
        
        if self.send_marker:
            self.route_publisher = self.create_publisher(PoseStamped, self.marker, 10)

    def publish_goal_dict(self, point) -> None:
        pose = PoseStamped()
        self.time_stamp = self.get_clock().now().to_msg()
        
        pose.header.frame_id = 'map'
        pose.header.stamp = self.time_stamp
        pose.pose.position.x = point['x']
        pose.pose.position.y = -point['y']
        pose.pose.position.z = point['z']
        quaternion = quaternion_from_euler(
            0, 0, -math.radians(point['yaw']))
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.route_pub_.publish(pose)
            
    def publish_goal(self, point) -> None:
        pose = PoseStamped()
        self.time_stamp = self.get_clock().now().to_msg()
        
        pose.header.frame_id = 'map'
        pose.header.stamp = self.time_stamp
        pose.pose.position.x = point[0].location.x
        pose.pose.position.y = -point[0].location.y
        pose.pose.position.z = point[0].location.z
        quaternion = quaternion_from_euler(
            0, 0, -math.radians(point[0].rotation.yaw))
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.route_pub_.publish(pose)

    def send_engage(self, state: bool) -> None:
        engage_msg = Engage()
        engage_msg.engage = state
        self.engage_pub_.publish(engage_msg)
        self.get_logger().info(f"Engaging {state}")
        self.state.sent_engage = state


    def plan_state_cb(self, route_msg: RouteState) -> None:
        """ Callback for ros2 topic
            /planning/mission_planning/state
        """
        curr = route_msg.state
        
        if not curr == self.state.planning_state:
            self.logger.info(f"Planning State: {curr}")
            self.state.planning_state = curr
            
    def traj_state_cb(self, traj_msg: Trajectory) -> None:
        """ Callback for ros2 topic
            /planning/scenario_planning/trajectory'
        """
        self.state.position = traj_msg.points[-1].pose.position
    
    def motion_state_cb(self, motion_msg: MotionState) -> None:
        """ Callback for ros2 topic
            /api/motion/state
        """
        curr = motion_msg.state
        
        if not curr == self.state.motion_state:
            self.logger.info(f"Motion State: {curr}")
            self.state.motion_state = curr
            
    def remaining_dist_cb_(self, time_distance_msg: MissionRemainingDistanceTime):
        self.state.distance_to_goal = time_distance_msg.remaining_distance
    
if __name__ == '__main__':
    agent = AutowareAgent()
    agent.setup()
    
    iterations = 10
    for i in range(0,iterations):
        time.sleep(1)
        agent.run_step(0,0)
        