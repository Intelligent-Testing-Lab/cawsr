import rclpy
import threading
import sys
import signal

# uncomment if testing in scenario runner
# from srunner.autoagents.autonomous_agent import AutonomousAgent
# class AutowareAgent(AutonomousAgent):

DEBUG_ENV = False

class AutowareAgent():
  
    timestamp = None
    current_map = None
    
    def setup(self, _path_to_conf: dict | None) -> None:
        """Setup the Autoware Agent.
            - Initialise the state
            - Setup nodes
        
        Args:
            _path_to_conf (dict | None): path to config, passed from AutonomousAgent
        """
        
        # define nodes
        self.route_node = None
        self.autoware_node = None
        self.state_node = None

        self.nodes = [self.route_node, self.autoware_node, self.state_node]
        self._node_threads = []
        
        
        return
    
    def destroy(self) -> None:
        for thread in self._node_threads:
            thread.join()
            
        for node in self.nodes:
            node.destroy_node()
        
        rclpy.shutdown()
    
    
    def run_step(self) -> None:
        return
    
    
    
if __name__ == '__main__':
    agent = AutowareAgent()
    agent.setup()
    
    if DEBUG_ENV:
        