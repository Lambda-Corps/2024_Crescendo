from commands2 import Subsystem
from phoenix5 import TalonSRX, TalonSRXControlMode
from wpilib import SmartDashboard


class ShooterTest(Subsystem):
    """
    Test class for shooter prototype
    """

    def __init__(self):
        super().__init__()

        self._leftwheel = TalonSRX(8)
        self._rightwheel = TalonSRX(9)

        self._leftwheel.configFactoryDefault()
        self._rightwheel.configFactoryDefault()

        SmartDashboard.putNumber("Left Wheel", 0)
        SmartDashboard.putNumber("Right Wheel", 0)

    def drive_motors(self, left: float, right: float):
        self._leftwheel.set(TalonSRXControlMode.PercentOutput, left)
        self._rightwheel.set(TalonSRXControlMode.PercentOutput, right)
