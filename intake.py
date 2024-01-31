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

        self._intakeroller = TalonSRX(constants.INTAKE_ROLLER)
        self._intakeroller.configFactoryDefault()
        self._indexroller = TalonSRX(constants.INDEX_ROLLER)
        self._indexroller.configFactoryDefault()
        self._indexleft = TalonSRX(constants.INDEX_LEFT)
        self._indexleft.configFactoryDefault()
        self._indexleft.setInverted(True)
        self._indexright = TalonSRX(constants.INDEX_RIGHT)
        self._indexright.configFactoryDefault()

        SmartDashboard.putNumber("IntakeSpeed", 0.3)

    def drive_index(self, speed: float):
        self._indexleft.set(TalonSRXControlMode.PercentOutput, speed)
        self._indexright.set(TalonSRXControlMode.PercentOutput, speed)
        self._indexroller.set(TalonSRXControlMode.PercentOutput, speed)
        self._intakeroller.set(TalonSRXControlMode.PercentOutput, speed)


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
        self._speed = SmartDashboard.getNumber("IntakeSpeed", 0.3)
        print(f"Intake Test Initialize")

    def execute(self):
        self._sub.drive_index(self._speed)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        print(f"Intake Test End")
        self._sub.drive_index(0)
