from srunner.objects.sensors import DefaultSensor

import carla


class EnvironmentConfig(object):
    """
    Simple object to store information about the initial environment setup for the scenario loop
    """

    def __init__(self) -> None:
        self.town: str = ""
        self.ego_model: str = ""
        self.ego_name: str = ""
        self.ego_spawn: carla.Transform | None = None
        self.sensor_config: list[DefaultSensor] = []
        self.route_id: int = 0
