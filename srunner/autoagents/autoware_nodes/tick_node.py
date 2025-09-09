# import message types

from rclpy.node import Node

from autoware_carla_interface_msgs.srv import AutowareTick


class TickNode(Node):
    """ROS2 Client Node solely responsible for ticking the Autoware-Carla-Bridge.
    The service is called syncronously, blocking execution until Autoware executes the action and replies.

    The node also has an optional boolean to enable tracking execution time.
    """

    tick_service = "autoware_tick"

    def __init__(self, exec_time: bool = False, debug: bool = False) -> None:
        super().__init__("tick_node_client")
        self._exec_time = exec_time
        self.debug = debug

        self.tick_client = self.create_client(AutowareTick, self.tick_service)

        self.get_logger().info(f"Waiting for '{self.tick_service}' service...")
        while not self.tick_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Service '{self.tick_service}' not available, waiting again..."
            )
        self.get_logger().info(f"Service '{self.tick_service}' available.")

        self.req = AutowareTick.Request()

    def autoware_tick(self) -> None:
        self.req.header.frame_id = "map"
        self.req.header.stamp = self.get_clock().now().to_msg()

        # send to service - blocking call
        res = self.tick_client.call(self.req)

        if self.debug:
            self.get_logger().info(
                f"Service {self.tick_service} responded with delta of {res.delta}ms"
            )
        if res.delta == 0.0:
            self.get_logger().info(
                f"Service {self.tick_service} responded with invalid time-delta. Is CARLA running?"
            )
