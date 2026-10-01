import carla


class DefaultSensor(object):
    """
    A class to encapsulate information about a basic sensor
    """

    def __init__(self) -> None:
        self.type: str = ""
        self.id: str = ""
        self.spawn: carla.Transform | None = None
        self.publish_frequency: int = 0

    def _spawn_to_dict(self) -> dict:
        if self.spawn is None:
            return {}
        return {
            "x": self.spawn.location.x,
            "y": self.spawn.location.y,
            "z": self.spawn.location.z,
            "pitch": self.spawn.rotation.pitch,
            "roll": self.spawn.rotation.roll,
            "yaw": self.spawn.rotation.yaw,
        }

    def sensor_dict(self) -> dict:
        """Converts all sensor attributes in a dictionary

        Returns:
            dict: Dictionary for sensor params in the format expected by autoware_carla_interface
        """
        sensor_config = {}

        ignored_attr = ["sensor_dict", "_spawn_to_dict"]
        sensor_params = [attr for attr in dir(self) if attr not in ignored_attr]
        sensor_params = filter(lambda x: not x.startswith("__"), sensor_params)

        for sensor_param in sensor_params:
            if sensor_param.lower() == "spawn":
                sensor_config["spawn_point"] = self._spawn_to_dict()
                continue
            sensor_config[sensor_param] = getattr(self, sensor_param)

        print(sensor_config)
        return sensor_config


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
