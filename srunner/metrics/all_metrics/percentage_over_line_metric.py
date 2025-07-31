import math

from .basic_metric import BasicMetric


class PercentageOverLineMetric(BasicMetric):
    
    def __init__(self, town_map, log, criteria=None):
        super().__init__(town_map, log, criteria)
        self.metric_value = 0
        
    def _create_metric(self, town_map, log, criteria):

        # Get ego vehicle id
        ego_id = log.get_ego_vehicle_id()

        # Get the frames the ego actor was alive and its transforms
        start, end = log.get_actor_alive_frames(ego_id)
        
        actor_over_line_counter = 0

        # Get the projected distance vector to the center of the lane
        for i in range(start, end + 1):

            ego_location = log.get_actor_transform(ego_id, i).location
            ego_waypoint = town_map.get_waypoint(ego_location)

            # Get the distance vector and project it
            a = ego_location - ego_waypoint.transform.location      # Ego to waypoint vector
            b = ego_waypoint.transform.get_right_vector()           # Waypoint perpendicular vector
            b_norm = math.sqrt(b.x * b.x + b.y * b.y + b.z * b.z)

            ab_dot = a.x * b.x + a.y * b.y + a.z * b.z
            dist_v = ab_dot/(b_norm*b_norm)*b
            dist = math.sqrt(dist_v.x * dist_v.x + dist_v.y * dist_v.y + dist_v.z * dist_v.z)

            # Get the sign of the distance (left side is positive)
            c = ego_waypoint.transform.get_forward_vector()         # Waypoint forward vector
            ac_cross = c.x * a.y - c.y * a.x
            if ac_cross < 0:
                dist *= -1

            if dist <= 0:
                actor_over_line_counter += 1
                
        total_frames = end - start
        percentage = (total_frames - actor_over_line_counter/actor_over_line_counter) * 100
        self.metric_value = 1 - percentage

    def get_value(self):
        return self.metric_value
        

            
        
    
