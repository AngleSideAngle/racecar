import abc
import time
from typing import Callable, Optional

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
    
    def and_then(self, other):
        return Sequence(self, other)
    
    def race_with(self, other):
        return ParallelRace(self, other)
    
    def with_timeout(self, delta_time: float):
        return self.race_with(WaitCommand(delta_time))
    
    def __add__(self, other):
        return self.and_then(other)

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

    def __init__(self, *commands: Command) -> None:
        self.commands = {cmd:True for cmd in commands}

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
    
class ParallelRace(Command):

    def __init__(self, *commands: Command) -> None:
        self.commands = {cmd:True for cmd in commands}
        self.finished = False

    def initialize(self) -> None:
        for command in self.commands:
            command.initialize()

    def execute(self) -> None:
        for command in self.commands:
            command.execute()
            if command.is_finished():
                self.finished = True

    def end(self, interrupted: bool) -> None:
        for command in self.commands:
            command.end(not command.is_finished())

    def is_finished(self) -> bool:
        return self.finished
    
class Scheduler:
    scheduled: Optional[Command]
    to_schedule: Optional[Command]
    default_command: Optional[Command]

    def __init__(self, default_command: Command = None) -> None:
        self.scheduled = None
        self.to_schedule = None
        self.default_command = default_command

    def schedule(self, command: Command) -> None:
        self.to_schedule = command

    def run(self):
        """
        Should be ran every tick
        """
        # runs scheduled command
        if self.scheduled:
            self.scheduled.execute()
            if self.scheduled.is_finished:
                self.scheduled.end(False)
                self.scheduled = None

        # schedules new command
        if self.to_schedule:
            if self.scheduled:
                self.scheduled.end(True)
            self.scheduled = self.to_schedule
            self.to_schedule = None

        # schedules default if needed
        if not self.scheduled and self.default_command:
            self.scheduled = self.default_command
            
# utility functions
def instant_command(action: Callable[[], None]) -> Command:
    return CommandImpl(execute=action, is_finished=lambda: True)

def print_command(text: str) -> Command:
    return instant_command(lambda: print(text))

    
# tests
if __name__ == "__main__":

    scheduler = Scheduler(default_command=print_command("default"))

    a = print_command("hi")
    b = print_command("bye")
    combined = a + b

    other = CommandImpl(execute=lambda: print("spam")).with_timeout(2.0) # spams for 2 seconds

    combined.execute()
    combined.execute()
    combined.execute()

    idk = CommandImpl(execute=lambda: print("executing"), end=lambda interrupted: print(f"interrupted: {interrupted}"))
    conflicting = print_command("the interrupting command")

    scheduler.schedule(idk)

    scheduler.run()

    scheduler.schedule(conflicting)

    scheduler.run()
    scheduler.run()
    scheduler.run()
    scheduler.run()

