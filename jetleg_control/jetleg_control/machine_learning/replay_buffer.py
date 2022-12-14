from collections import deque

import random

class ReplayBuffer:
    
    def __init__(self, maxlen):
        self.maxlen = maxlen
        self.buffer = []
        self.insert_pos = 0

    def add(self, element):
        if len(self.buffer) < self.maxlen:
            self.buffer.append(element) # popleft if maxlen is reached
        else:
            self.buffer[self.insert_pos] = element
            self.insert_pos = (self.insert_pos + 1) % len(self.buffer)

    def get_training_samples(self, size):
        mini_sample = []
        if len(self.buffer) > size:
            mini_sample = random.sample(self.buffer, size) # list of tuples
        else:
            mini_sample = self.buffer

        return mini_sample