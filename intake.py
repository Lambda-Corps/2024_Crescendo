from commands2 import Subsystem, Command
from phoenix5 import TalonSRX, TalonSRXControlMode
from wpilib import SmartDashboard

import constants


class Intake(Subsystem):
    """
    Test class for shooter prototype
    """

    def __init__(self):
        super().__init__()

        self._motor = TalonSRX(constants.INTAKE_MOTOR)

        self._motor.configFactoryDefault()

        SmartDashboard.putNumber("IntakeSpeed", 0)

    def drive_motor(self, speed: float):
        self._motor.set(TalonSRXControlMode.PercentOutput,speed)


class IntakeTestCommand(Command):
    """
    Command to run motors of the shooter with a button press
    """

    def __init__(self, intake: Intake):
        super().__init__()
        self._speed = 0
        self._sub = intake

        self.addRequirements(self._sub)

    def initialize(self):
        self._speed = SmartDashboard.getNumber("IntakeSpeed", .5)
        print(f"Shooter Test Initialize")

    def execute(self):
        self._sub.drive_motor(self._speed)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        print(f"Shooter Test End")
        self._sub.drive_motor(0)
