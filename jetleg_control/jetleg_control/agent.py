import torch
import random
from collections import deque
from model import Linear_QNet, QTrainer

import rclpy
import rclpy.qos;
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

MAX_MEMORY = 100 * 1000
BATCH_SIZE = 1000
LR = 0.001

class Agent(Node):
    
    def __init__(self):
        super().__init__('single_leg_stabilizer')

        # Topics to make subscriptions
        self.joints_topic = self.get_parameter('joints_topic').value
        self.link_topic = self.get_parameter('links_topic').value

        # Subscription to joint states
        self.joints_subscription = self.create_subscription(JointState, self.joints_topic, self.update_joint_states, rclpy.qos.qos_profile_system_default)
        # Subscription to link states
        self.links_subscription = self.create_subscription(PoseStamped, self.joints_topic, self.update_link_state, rclpy.qos.qos_profile_system_default)

        # Stores most current joint states
        self.joints = {}
        # Stores most current link states
        self.links = {}

        # Number of joints in model
        self.num_joints = 2
        # Number of parameters per joint (position, velocity, effort)
        self.per_joint_data = 3

        # Number of links
        self.num_links = 2
        # Number of parameters per link (positionx3 orientationx4)
        self.per_link_data = 7

        # Total number of parameters for all joints and links
        self.num_state_params = self.num_joints * self.per_joint_data + self.num_links * self.per_link_data

        self.n_games = 0
        self.epsilon = 0 # randomness
        self.gamma = 0.9 # discount rate
        self.memory = deque(maxlen=MAX_MEMORY)
        self.model = Linear_QNet(self.num_state_params, 512, self.num_joints)
        self.trainer = QTrainer(self.model, lr=LR, gamma=self.gamma)

    def update_joint_states(self, msg):
        """Update link state from subscription"""

        for i in range(len(msg.name)):
            self.joints[msg.name] = [self.msg.position[i], self.msg.velocity[i], self.msg.effort[i]]

    def update_link_state(self, msg):
        """Update link state from subscription"""

        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        self.links[msg.header.frame_id] = [position, orientation]

    def get_joint_states(self):
        joint_states = []
        for joint_name in self.joints: # Iterates through all joints

            joint_state = []
            for joint_param in self.joints[joint_name]:
                joint_state += joint_param

            joint_states += joint_state

        return joint_states

    def get_link_states(self):
        link_states = []
        for link_name in self.links: # Iterates through all joints

            link_state = []
            for link_param in self.links[link_name]:
                link_state += link_param
                
            link_states += link_state

        return link_states

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
        joint_states = self.get_joint_states()
        link_states = self.get_link_states()

        state = joint_states + link_states

        # get move
        next_move = self.get_action(state)
        # process move
        reward, done, score = self.act(next_move)

        # get new state
        new_state = self.update_joint_states(msg)

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