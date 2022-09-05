import torch
import random
from collections import deque
from model import Linear_QNet, QTrainer

import rclpy
import rclpy.qos;
from rclpy.node import Node
from sensor_msgs.msg import JointState

MAX_MEMORY = 100 * 1000
BATCH_SIZE = 1000
LR = 0.001

class Agent(Node):
    
    def __init__(self):
        super().__init__('single_leg_stabilizer')

        # Topics to make subscription
        self.topic = self.get_parameter('joints_topic').value
        # Subscription to joint states
        self.subscription = self.create_subscription(JointState, self.topic, self.train, rclpy.qos.qos_profile_system_default)
        
        # Stores most current joint states
        self.joints = {}

        # Number of joints in model
        self.num_joints = 2
        # Number of parameters per joint (position, velocity, effort)
        self.per_joint_data = 3
        # Total number of parameters for all joints
        self.num_state_params = self.num_joints * self.per_joint_data

        self.n_games = 0
        self.epsilon = 0 # randomness
        self.gamma = 0.9 # discount rate
        self.memory = deque(maxlen=MAX_MEMORY)
        self.model = Linear_QNet(self.num_state_params, 512, self.num_joints)
        self.trainer = QTrainer(self.model, lr=LR, gamma=self.gamma)
    
    def update_state(self, msg):
        state = []

        for i in range(len(msg.name)):
            self.joints[msg.name] = [self.msg.position[i], self.msg.velocity[i], self.msg.effort[i]]
            state += self.joints[msg.name]

        return state

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done)) # popleft if MAX_MEMORY is reached


    def train_long_memory(self):
        if len(self.memory) > BATCH_SIZE:
            mini_sample = random.sample(self.memory, BATCH_SIZE) # list of tuples
        else:
            mini_sample = self.memory

        states, actions, rewards, next_states, dones = zip(*mini_sample)
        self.trainer.train_step(states, actions, rewards, next_states, dones)

    def train_short_memory(self, state, action, reward, next_state, done):
        self.trainer.train_step(state, action, reward, next_state, done)

    def get_action(self, state):
        # random moves: tradeoff between exploration / exploitation
        self.epsilon = 80 - self.n_games
        final_move = [0, 0, 0]
        if random.randint(0, 200) < self.epsilon:
            move = random.randint(0, 2)
            final_move[move] = 1
        else:
            state0 = torch.tensor(state, dtype=torch.float)
            prediction = self.model(state0)
            move = torch.argmax(prediction).item()
            final_move[move] = 1
        
        return final_move

    def act(self, move):
        pass

    def train(self, msg):
        # get state
        state = self.update_state(msg)

        # get move
        next_move = self.get_action(state)
        # process move
        reward, done, score = self.act(next_move)

        # get new state
        new_state = self.update_state(msg)

        # train short memory
        self.train_short_memory(state, next_move, reward, new_state, done)

        # remember
        self.remember(state, next_move, reward, new_state, done)

        if done:
            # train long memory, plot results
            self.n_games += 1
            self.train_long_memory()

            if score > record:
                record = score
                self.model.save()


if __name__ == '__main__':
    # Initialize ROS 2
    rclpy.init()

    # Create and run agent training
    agent = Agent()
    rclpy.spin(agent)

    # Release node resources and shutdown ROS
    agent.destroy_node()
    rclpy.shutdown()