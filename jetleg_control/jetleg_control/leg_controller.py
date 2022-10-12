import agent
import model


class Leg_Controller(Agent):

    def get_state(self,simulation):
        state = [simulation.last_joint_state,simulation.last_link_state]
        return state

    def get_action(self,state):
        # random moves: tradeoff between exploration / exploitation
        # self.epsilon = 200 - self.n_games
        final_move = [0, 0]
        if random.randint(0, 200) < self.epsilon:
            move = random.randint(0, 1)
            final_move[move] = 1
        else:
            state0 = torch.tensor(state, dtype=torch.float)
            prediction = self.model.forward(state0)
            move = torch.argmax(prediction).item()
            final_move[move] = 1
        
        return np.array(final_move, dtype=int)
    
    def __init__(self):
        super()__init__()
        self.model = Linear_QNet([11, 1024, 2])
        self.trainer = QTrainer(self.model, lr=LR, gamma=self.gamma)
        