from srunner.autoagents.agent_state import state


class AutowareState(state.AgentState):
    def __init__(self, name="", position=None):
        super().__init__(name, position)

        self.reset_state()

    def within_goal(self) -> bool:
        return (
            self.remaining_distance >= self.goal_threshold
            or self.remaining_distance == self.goal_threshold
        ) and (not (self.current_waypoint + 1) == self.total_waypoints)

    def completed_route(self) -> bool:
        return self.route_state == 6 or self.motion_state == 1

    def route_set(self) -> bool:
        return self.route_state == 4

    def no_route(self) -> bool:
        return self.route_state == 2 or self.route_state == 0  # debug

    def has_localized(self) -> bool:
        return self.localize_state == 3

    def is_ready_publish_route(self) -> bool:
        return (not self.sent_route) and self.no_route() and self.has_localized()

    def is_planning(self) -> bool:
        return self.route_state == 3

    def route_ready(self) -> bool:
        return self.sent_route and self.route_set()

    def reset_state(self) -> None:
        # internal message states
        self.sent_route: bool = False
        self.sent_engage: bool = False
        self.bridge_ready: bool = False

        # ADS state
        self.motion_state: int = 0
        self.route_state: int = 0
        self.localize_state: int = 0

        # goal state
        self.goal_threshold: float = 5.0
        self.initial_distance: float = -1.0  # distance reading when ADS reaches goal
        self.remaining_distance: float = -1.0

        self.current_waypoint = 0
        self.total_waypoints = 0
