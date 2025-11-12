import logging

logger = logging.getLogger("scenario-runner")


class TM:
    """Config class encompassing CARLA Traffic Manager parameters"""

    SEED = 0
    PORT = 8001


class CARLA:
    """Config class encompassing CARLA parameters"""

    HOST = "127.0.01"
    FIDELITY = "High"
    SYNC = True
    PORT = 2000
    TIMEOUT = 20
    FIXED_DELTA_SECONDS = 0.05
    TRAFFIC_MANAGER = TM()

    def _parse_dict(self, obj, conf_dict: dict) -> None:
        """Updates the CARLA class attributes based on the config file

        Args:
            conf_dict (dict): new attribute values
        """
        keys = conf_dict.keys()

        for key in keys:
            try:
                if key == "traffic_manager":
                    tm_config = TM()
                    tm_config.PORT = conf_dict[key]["port"]
                    tm_config.SEED = conf_dict[key]["seed"]

                    conf_dict[key] = tm_config

                setattr(obj, key.upper(), conf_dict[key])
            except AttributeError:
                logger.error(
                    f"Cannot set attribute {key} for CARLA config, does the attribute exist?"
                )
