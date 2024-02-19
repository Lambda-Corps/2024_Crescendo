from commands2 import Subsystem, Command, cmd
from phoenix5 import TalonSRX, TalonSRXControlMode
from wpilib import SmartDashboard, AnalogInput, RobotBase

import constants


class Climber(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._left_climber: TalonSRX = TalonSRX(constants.LEFT_CLIMBER)
        self._left_climber.configFactoryDefault()

        self._right_climber: TalonSRX = TalonSRX(constants.RIGHT_CLIMBER)
        self._right_climber.configFactoryDefault()

    def drive_climber_up(self) -> None:
        self._left_climber.set(TalonSRXControlMode.PercentOutput, 1)
        self._right_climber.set(TalonSRXControlMode.PercentOutput, 1)

    def drive_climber_down(self) -> None:
        self._left_climber.set(TalonSRXControlMode.PercentOutput, -1)
        self._right_climber.set(TalonSRXControlMode.PercentOutput, -1)

    def climber_done(self) -> bool:
        left_amps: float = self._left_climber.getStatorCurrent()
        right_amps: float = self._right_climber.getStatorCurrent()
