from .basic_metric import BasicMetric

class AccelerationMetric(BasicMetric):
    def __init__(self, town_map, log, criteria=None):
        super().__init__(town_map, log, criteria)
        self.metric_value = 0
        
    def _create_metric(self, town_map, log, criteria):
        ego_id = log.log.get_ego_vehicle_id()
        start, end = log.get_actor_alive_frames(ego_id)
        
        biggest_difference = 0
        prev_frame = start
        for frame in range(start, end + 1):
            prev_accel = log.get_actor_acceleration(ego_id, prev_frame)
            curr_accel = log.get_actor_acceleration(ego_id, frame)
            accel_diff = abs(curr_accel - prev_accel)
            
            if accel_diff > biggest_difference:
                biggest_difference = accel_diff
                
                
        self.metric_value = biggest_difference
        
    
    def get_value(self):
        return self.metric_value