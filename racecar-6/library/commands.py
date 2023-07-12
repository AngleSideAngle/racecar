from abc import ABC, abstractmethod
from typing import Callable

class Command(ABC):

    @abstractmethod
    def initialize(self) -> None:
        pass

    @abstractmethod
    def execute(self) -> None:
        pass

    @abstractmethod
    def end(self, interrupted: bool) -> None:
        pass

    @abstractmethod
    def is_finished(self) -> bool:
        return False
    
    def __add__(self, other):
        return Sequence(self, other)

class CommandImpl(Command):

    def __init__(
        self,
        initialize: Callable[[], None] = lambda: None,
        execute: Callable[[], None] = lambda: None,
        end: Callable[[bool], None] = lambda interrupted: None,
        is_finished: Callable[[], bool] = lambda: False
    ) -> None: 
        self.initialize = initialize
        self.execute = execute
        self.end = end
        self.is_finished = is_finished

    def initialize(self) -> None:
        self.initialize()

    def execute(self) -> None:
        self.execute()

    def end(self, interrupted: bool) -> None:
        self.end(interrupted)

    def is_finished(self) -> bool:
        return self.is_finished()

class Sequence(Command):

    def __init__(self, *commands: Command) -> None:
        self.commands = commands
        self.index = -1

    def initialize(self) -> None:
        if len(self.commands) > 0:
            self.commands[0].initialize()
        self.index = 0

    def execute(self) -> None:
        if len(self.commands) < 1:
            return
        
        current_command = self.commands[self.index]

        current_command.execute()

        if current_command.is_finished():
            current_command.end(interrupted=False)
            self.index =+ 1
            if self.index < len(self.commands):
                self.commands[self.index].initialize()

    def end(self, interrupted: bool) -> None:
        if interrupted and self.index > -1 and self.index < len(self.commands):
            self.commands[self.index].end()
        self.index = -1

    def is_finished(self) -> bool:
        return self.index == len(self.commands)
    
# tests
if __name__ == "__main__":
    a = CommandImpl(execute=lambda: print("hi"), is_finished=lambda: True)
    b = CommandImpl(execute=lambda: print("bye"), is_finished=lambda: True)
    combined = a + b
    combined.execute()
    combined.execute()
