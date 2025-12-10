from abc import ABC, abstractmethod
from geometry_msgs.msg import Twist

class ControlAlgorithm(ABC):
    """Base class for control algorithms producing Twist in [-1,1]."""
    def __init__(self, params: dict):
        self.params = params or {}

    @abstractmethod
    def update(self, now_sec: float) -> Twist:
        ...

    def on_state(self, state_msg):
        pass
