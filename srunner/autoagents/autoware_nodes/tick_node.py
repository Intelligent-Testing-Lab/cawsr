# import message types

from rclpy.node import Node


class TickNode(Node):
    """ROS2 Client Node solely responsible for ticking the Autoware-Carla-Bridge.
    The service is called syncronously, blocking execution until Autoware executes the action and replies.

    The node also has an optional boolean to enable tracking execution time.
    """

    tick_service = "/autoware/tick"

    def __init__(self, exec_time: bool = False, debug: bool = False) -> None:
        super().__init__("")
        self._exec_time = exec_time
        self.debug = debug

        self.tick_client = self.create_client(
            # message type - implement,
            self.tick_service
        )

    def autoware_tick(self) -> None:
        # get current time
        # assemble tick message
        # over client
        # block until response received

        return
