from abc import ABCMeta

import numpy as np


class ClassifierInterface(metaclass=ABCMeta):
    """Interface defining the common methods of a state estimator"""
    
    @classmethod
    def __subclasshook__(cls, __subclass: type) -> bool:
        return (
            hasattr(__subclass, "filter") and
            callable(__subclass.filter)
        )
    
class KalmanFilter:
    """State estimator which uses the kalman filter algorithm for predicting state"""

    def __init__(self, A: np.array, B: np.array):
        pass

    def filter(self):
        pass

    def forward(self):
        pass