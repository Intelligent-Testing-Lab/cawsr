from srunner.autoagents.agent_state import state

class AutowareState(state.AgentState):
    
    def __init__(self, name="", position=None):
        super().__init__(name, position)
        
        self.reset_state()
        
    def within_goal(self) -> None:
        return
    
    def completed_route(self) -> None:
        return    
        
    def route_set(self) -> None:
        return
    
    def no_route(self) -> None:
        return
    
    def is_planning(self) -> None:
        return
    
    def reset_state(self) -> None:
        # internal message states
        self.sent_route: bool = False
        self.sent_engage: bool = False
        
        # ADS state
        self.motion_state: int= 0
        self.route_state: int = 0
        self.localize_state: int = 0
        
        # goal state
        self.achieved_goal: bool = False
        self.distance_to_goal: float = 0
        self.goal_threshold: float = 0.5 
    
        self.initial_distance: float = -1 # distance reading when ADS reaches goal
        
        
        
