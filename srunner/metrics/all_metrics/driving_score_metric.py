import importlib
from .basic_metric import BasicMetric

class DrivingScore(BasicMetric):
    def __init__(self, town_map, log, metrics, criteria=None):
        super().__init__(town_map, log, criteria)
        self.metric_value = 0       
        self.metrics = metrics
        
    def _create_metric(self, town_map, log, criteria):
        metric_values = []
        module = importlib.import_module('srunner.metrics.all_metrics')
        
        for metric in self.metrics:
            instance = getattr(module, metric)(town_map, log, criteria)
            metric_values.append(instance.get_value())
            
        self.metric_value = sum(metric_values)
        
    def get_value(self):
        return self.metric_value
            
    