from commands2 import Subsystem, Command
from wpilib import SmartDashboard


class Shooter(Subsystem):
    """
    Test class for shooter prototype
    """

    def __init__(self):
        super().__init__()

        SmartDashboard.putNumber("ShooterSpeed", 0)

    def drive_motors(self, speed: float):
        pass


class ShooterTestCommand(Command):
    """
    Command to run motors of the shooter with a button press
    """

    def __init__(self, shooter: Shooter):
        super().__init__()
        self._shootspeed = 0
        self._sub = shooter

        self.addRequirements(self._sub)

    def initialize(self):
        self._shootspeed = SmartDashboard.getNumber("ShooterSpeed", 0)
        print(f"Shooter Test Initialize")

    def execute(self):
        self._sub.drive_motors(self._shootspeed)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        print(f"Shooter Test End")
        self._sub.drive_motors(0)
