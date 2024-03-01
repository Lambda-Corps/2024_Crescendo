from commands2 import Subsystem, Command, cmd
from phoenix5 import TalonSRX, TalonSRXControlMode
from wpilib import SmartDashboard, AnalogInput, RobotBase, Timer

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

    def stop_climber_motors(self) -> None:
        self._left_climber.set(TalonSRXControlMode.PercentOutput, 0)
        self._right_climber.set(TalonSRXControlMode.PercentOutput, 0)


class MoveClimber(Command):
    def __init__(self, sub: Climber, speed: float, timeout=0):
        super().__init__()

        self._speed = speed
        self._climber = sub
        self._timeout = timeout

        self._timer = Timer()
        self._timer.start()

        self.addRequirements(self._climber)

    def initialize(self):
        self._timer.restart()

    def execute(self):
        if self._speed > 0:
            self._climber.drive_climber_up()
        elif self._speed < 0:
            self._climber.drive_climber_down()
        else:
            self._climber.stop_climber_motors()

    def isFinished(self) -> bool:
        if self._timeout == 0:
            return False

        return self._timer.hasElapsed(self._timeout)

    def end(self, interrupted: bool):
        self._climber.stop_climber_motors()
