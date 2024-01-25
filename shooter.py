from commands2 import Subsystem, Command
from phoenix5 import TalonSRX, TalonSRXControlMode
from wpilib import SmartDashboard


class Shooter(Subsystem):
    """
    Test class for shooter prototype
    """

    def __init__(self):
        super().__init__()

        # self._leftwheel = TalonSRX(8)
        # self._rightwheel = TalonSRX(9)

        # self._leftwheel.configFactoryDefault()
        # self._rightwheel.configFactoryDefault()

        SmartDashboard.putNumber("Left Wheel", 0)
        SmartDashboard.putNumber("Right Wheel", 0)

    def drive_motors(self, left: float, right: float):
        # self._leftwheel.set(TalonSRXControlMode.PercentOutput, left)
        # self._rightwheel.set(TalonSRXControlMode.PercentOutput, right)
        pass


class ShooterTestCommand(Command):
    """
    Command to run motors of the shooter with a button press
    """

    def __init__(self, shooter: Shooter):
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
