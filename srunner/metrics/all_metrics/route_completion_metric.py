from math import sqrt, pow
from .basic_metric import BasicMetric
from srunner.metrics.helper.metric_data import MetricData


class RouteCompletionMetric(BasicMetric):
    
    def __init__(self, town_map, log, criteria=None):
        super().__init__(town_map, log, criteria)
        self.metric_value = 0
        
    def _create_metric(self, town_map, log, criteria):
        ego_id = log.get_ego_vehicle_id()
        start, end = log.get_actor_alive_frames(ego_id)
        
        waypoint_index = 0
        ego_waypoints = MetricData.definition['routes'][0]['waypoints']
        for i in range(start, end + 1):
            curr_waypoint = ego_waypoints[waypoint_index]['position']
            ego_location = log.get_actor_transform(ego_id, i).location
            
            if self._euclidean_distance(curr_waypoint, ego_location) < 0.1:
                waypoint_index += 1
                self.metric_value = waypoint_index/len(ego_waypoints)
        
    def _euclidean_distance(self, curr_waypoint, ego_location):
        return sqrt(
            pow(curr_waypoint['x'] - ego_location.x, 2) +
            pow(curr_waypoint['y'] - ego_location.y, 2) +
            pow(curr_waypoint['z'] - ego_location.z, 2)
        )
    
    def get_value(self):
        return self.metric_value