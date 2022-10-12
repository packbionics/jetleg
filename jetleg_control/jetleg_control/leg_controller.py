import agent

class Leg_Controller(Agent):

    def get_state(self,simulation):
        state = [simulation.last_joint_state,simulation.last_link_state]
        return state
        
