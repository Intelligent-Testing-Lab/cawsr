from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

import carla


class DefaultSensor(object):
    """
    A class to encapsulate information about a basic sensor
    """

    def __init__(self) -> None:
        self.type: str = ""
        self.id: str = ""
        self.spawn: carla.Transform | None = None

    def _spawn(self, bp_library, vehicle) -> None:
        sensor_bp = bp_library.find(str(self.type))

        ignored_params = ["serealize", "id", "_spawn", "type", "spawn"]

        sensor_params = [attr for attr in dir(self) if attr not in ignored_params]
        sensor_params = filter(lambda x: not x.startswith("__"), sensor_params)

        for param in sensor_params:
            sensor_bp.set_attribute(param, str(getattr(self, param)))

        CarlaDataProvider.get_world().spawn_actor(sensor_bp, self.spawn, vehicle)
        CarlaDataProvider.get_world().tick()

    def serealize(self):
        return self.type, self.id


class CameraRGB(DefaultSensor):
    """
    A class to hold additional information about a camera sensor
    """

    def __init__(self) -> None:
        super().__init__()
        self.image_size_x: int = 0
        self.image_size_y: int = 0
        self.fov: float = 0.0


class LidarRayCast(DefaultSensor):
    """
    A class to hold additional information about a Lidar sensor
    """

    def __init__(self) -> None:
        super().__init__()
        self.range: int = 0
        self.channels: int = 0
        self.points_per_second: int = 0
        self.upper_fov: float = 0.0
        self.lower_fov: float = 0.0
        self.rotation_frequency: int = 0


class SensorGNSS(DefaultSensor):
    """
    A class to hold additional information about GNSS
    """

    def __init__(self) -> None:
        super().__init__()
        self.noise_alt_stddev = 0.0
        self.noise_lat_stddev = 0.0
        self.noise_lon_stddev = 0.0
        self.noise_alt_bias = 0.0
        self.noise_lat_bias = 0.0
        self.noise_lon_bias = 0.0


class SensorIMU(DefaultSensor):
    """
    A class to hold additional information about IMU
    """

    def __init__(self) -> None:
        super().__init__()
        self.noise_accel_stddev_x = 0.0
        self.noise_accel_stddev_y = 0.0
        self.noise_accel_stddev_z = 0.0
        self.noise_gyro_stddev_x = 0.0
        self.noise_gyro_stddev_y = 0.0
        self.noise_gyro_stddev_z = 0.0
