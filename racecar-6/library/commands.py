import abc
import time
from typing import Callable

class Command(abc.ABC):

    @abc.abstractmethod
    def initialize(self) -> None:
        pass

    @abc.abstractmethod
    def execute(self) -> None:
        pass

    @abc.abstractmethod
    def end(self, interrupted: bool) -> None:
        pass

    @abc.abstractmethod
    def is_finished(self) -> bool:
        return False
    
    def and_then(self, *other):
        return Sequence(self, other)
    
    def with_timeout(self, delta_time: float):
        pass
    
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
    
class WaitCommand(Command):

    def __init__(self, delta_time: float) -> None:
        self.dt = delta_time

    def initialize(self) -> None:
        self.start_time = time.perf_counter

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def is_finished(self) -> bool:
        return time.perf_counter > self.start_time + self.dt

class Sequence(Command):

    def __init__(self, *commands: Command) -> None:
        self.commands = list(commands)

    def initialize(self) -> None:
        if self.commands:
            self.commands[0].initialize()

    def execute(self) -> None:
        if not self.commands:
            return
        
        current_command = self.commands[0]

        current_command.execute()

        if current_command.is_finished():
            current_command.end(interrupted=False)
            self.commands.pop(0)
            if self.commands:
                self.commands[0].initialize()

    def end(self, interrupted: bool) -> None:
        if interrupted and self.commands:
            self.commands[0].end()

    def is_finished(self) -> bool:
        return not self.commands
    
class Parallel(Command):

    def __init__(self, deadline_behaviour: bool = False, *commands: Command) -> None:
        self.commands = {cmd:True for cmd in commands}
        self.deadline_behaviour = deadline_behaviour

    def initialize(self) -> None:
        for command in self.commands:
            command.initialize()

    def execute(self) -> None:
        for command, running in self.commands:
            if not running:
                continue
            command.execute()
            if command.is_finished():
                command.end(False)
                self.commands[command] = False
                


    def end(self, interrupted: bool) -> None:
        if interrupted:
            for command, running in self.commands:
                if running:
                    command.end(True)

    def is_finished(self) -> bool:
        return True not in self.commands.values()

    
# tests
if __name__ == "__main__":
    a = CommandImpl(execute=lambda: print("hi"), is_finished=lambda: True)
    b = CommandImpl(execute=lambda: print("bye"), is_finished=lambda: True)
    combined = a + b
    combined.execute()
    combined.execute()
    combined.execute()
