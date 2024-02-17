from commands2 import Command
from wpilib import Timer
from intake import Intake
from shooter import Shooter


class ShootCommand(Command):
    """
    Command to shoot a note that is currently held in the indexer

    Runs for the specified amount of time RUN_DURATION
    """

    RUN_DURATION = 2

    def __init__(self, intake: Intake, shooter: Shooter):
        super().__init__()

        self._intake = intake
        self._shooter = shooter

        self._timer = Timer()

        self.addRequirements(self._intake, self._shooter)

    def initialize(self):
        self._timer.reset()
        self._timer.start()
        # spin the shooter up to speed
        self._shooter.drive_motors()

    def execute(self):
        # Once the shooter is up to speed, index the notes
        if self._shooter.shooter_at_speed():
            self._intake.drive_index()

    def isFinished(self) -> bool:
        return self._timer.hasElapsed(self.RUN_DURATION)

    def end(self, interrupted: bool):
        self._shooter.stop_motors()
        self._intake.stop_indexer()
