from collections import deque

import random

class ReplayBuffer:
    
    def __init__(self, maxlen):
        self.buffer = deque(maxlen=maxlen)

    def add(self, element):
        self.buffer.append(element) # popleft if maxlen is reached

    def get_training_samples(self, size):
        mini_sample = []
        if len(self.buffer) > size:
            mini_sample = random.sample(self.buffer, size) # list of tuples
        else:
            mini_sample = self.buffer

        return mini_sample