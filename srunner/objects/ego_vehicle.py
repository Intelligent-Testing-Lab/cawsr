from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.tools.environment_parser import EnvironmentConfig

import carla
import logging

logger = logging.getLogger("scenario-runner")


class EgoVehicle(object):
    """
    A basic class to encompass the definition of an ego_vehicle controlled by ADS
    """

    ego_name = ""
    ego_model = ""
    world = None

    def __init__(self, env_config: EnvironmentConfig) -> None:
        self._env = env_config
        self.ego_model = self._env.ego_model
        self.ego_name = self._env.ego_name
        self.ego_spawn = self._env.ego_spawn

        self._actor = None

    def spawn(self) -> carla.Actor:
        """Spawns an actor to act as ego_vehicle

        Returns:
            carla.Actor: EgoVehicle class
        """

        self._actor = CarlaDataProvider.request_new_actor(
            self.ego_model, self.ego_spawn, self.ego_name
        )

        if self._actor is None:
            logger.warning(
                "Failed to spawn EgoVehicle. This is likely an issue with CARLA."
            )

        return self._actor  # type: ignore

    def prepare_ego(self, route_loc: carla.Transform) -> None:
        """Reset the position and velocity of the ego. Register the actor"""
        route_loc.location.z += 0.5
        self._actor.set_transform(route_loc)
        self._actor.set_target_velocity(carla.Vector3D())
        self._actor.set_target_angular_velocity(carla.Vector3D())

    def __del__(self) -> None:
        """Clean up
        Check if the actor exists in CARLA, delete if so
        """
        if self._actor.is_alive:
            self._actor.destroy()
