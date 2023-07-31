import abc

class State(abc.ABC):

    @abc.abstractmethod
    def execute(self) -> tuple[float, float]:
        pass

    @abc.abstractmethod
    def next_state(self) -> State:
        return self
