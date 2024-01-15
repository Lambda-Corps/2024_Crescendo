from commands2 import Command, Subsystem
from wpilib import SmartDashboard

from shooter_test import ShooterTest


class ShooterTestCommand(Command):
    """
    Command to run motors of the shooter with a button press
    """

    def __init__(self, shooter: ShooterTest):
        super().__init__()
        self._leftspeed = 0
        self._rightspeed = 0
        self._sub = shooter

        self.addRequirements(self._sub)

    def initialize(self):
        self._leftspeed = SmartDashboard.getNumber("Left Wheel", 0)
        self._rightspeed = SmartDashboard.getNumber("Right Wheel", 0)
        print(f"Shooter Test Initialize")

    def execute(self):
        self._sub.drive_motors(self._leftspeed, self._rightspeed)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        print(f"Shooter Test End")
        self._sub.drive_motors(0, 0)
