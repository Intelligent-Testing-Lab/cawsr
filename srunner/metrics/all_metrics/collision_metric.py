from .basic_metric import BasicMetric

class CollisionMetric(BasicMetric):
    
    def __init__(self, town_map, log, criteria=None):
        super().__init__(town_map, log, criteria)
        self.metric_value = 0
        
    def _create_metric(self, town_map, log, criteria):
        ego_id = log.get_ego_vehicle_id()
        collisions = log.get_actor_collisions(ego_id)
        self.metric_value = len(collisions)
    
    def get_value(self):
        return self.metric_value