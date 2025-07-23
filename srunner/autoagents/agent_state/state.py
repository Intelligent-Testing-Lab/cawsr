from geometry_msgs.msg import Pose


class AgentState(object):
    """Basic AgentState class to keep track of ROS agent planning and routing state. Inherit, override methods.
    See autoware_state.py for example
    """

    def __init__(self, name: str = "", position: Pose | None = None):
        """__init__ function for a basic ROS agent state.

        Args:
            name (str, optional): Name of the ROS agent. Defaults to "".
            position (Pose | None, optional): Initial Pose of the agent. Defaults to None.
        """
        self.name = name
        self.position = position

    def __repr__(self):
        """Calls __str__, returning the same representation of the state"""
        return self.__str__()

    def __str__(self):
        """Return name of state and list of attributes"""
        return f"{self.__class__.__name__} state: {self.__dict__}"

    def route_ready(self) -> None:
        pass
