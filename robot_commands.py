from commands2 import Command, InstantCommand
from wpilib import Timer
from intake import Intake
from shooter import Shooter


class ShootCommand(Command):
    """
    Command to shoot a note that is currently held in the indexer

    Runs for the specified amount of time RUN_DURATION
    """

    RUN_DURATION = 1.1

    def __init__(self, intake: Intake, shooter: Shooter):
        super().__init__()

        self._intake = intake
        self._shooter = shooter

        self._timer = Timer()

        self.addRequirements(self._intake, self._shooter)

    def initialize(self):
        self._timer.reset()
        self._timer.start()
        self._intake.set_shooting_flag(True)
        # spin the shooter up to speed
        self._shooter.drive_motors()

    def execute(self):
        # Once the shooter is up to speed, index the notes
        # if self._shooter.shooter_at_speed():
        if self._timer.hasElapsed(0.7):
            self._intake.drive_index(shooting=True)

    def isFinished(self) -> bool:
        return self._timer.hasElapsed(self.RUN_DURATION)

    def end(self, interrupted: bool):
        self._shooter.stop_motors()
        self._intake.stop_indexer()
        self._intake.set_shooting_flag(False)


class StopIndexAndShooter(InstantCommand):
    def __init__(self, shooter: Shooter, intake: Intake):
        super().__init__()

        self._shooter = shooter
        self._intake = intake

        self.addRequirements(self._shooter, self._intake)

    def end(self, interrupted: bool):
        self._shooter.stop_motors()
        self._intake.stop_indexer()
        self._intake.set_shooting_flag(False)


class DoubleShootCommand(Command):
    """
    Command to shoot a note that is currently held in the indexer

    Runs for the specified amount of time RUN_DURATION
    """

    RUN_DURATION = 2.5

    def __init__(self, intake: Intake, shooter: Shooter):
        super().__init__()

        self._intake = intake
        self._shooter = shooter

        self._timer = Timer()

        self.addRequirements(self._intake, self._shooter)

    def initialize(self):
        self._timer.reset()
        self._timer.start()
        self._intake.set_shooting_flag(True)
        # spin the shooter up to speed
        self._shooter.drive_motors()

    def execute(self):
        # Once the shooter is up to speed, index the notes
        # if self._shooter.shooter_at_speed():
        if self._timer.hasElapsed(0.7):
            self._intake.drive_index(shooting=True)

        if self._timer.hasElapsed(1.5):
            self._intake.drive_index(shooting=True, doubleshooting=True)

    def isFinished(self) -> bool:
        return self._timer.hasElapsed(self.RUN_DURATION)

    def end(self, interrupted: bool):
        self._shooter.stop_motors()
        self._intake.stop_indexer()
        self._intake.set_shooting_flag(False)
