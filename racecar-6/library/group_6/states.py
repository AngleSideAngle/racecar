import abc
from typing import Tuple

class State(abc.ABC):

    @abc.abstractmethod
    def execute(self) -> Tuple[float, float]:
        pass

    @abc.abstractmethod
    def next_state(self):
        return self
