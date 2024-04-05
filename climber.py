from commands2 import Subsystem, Command, cmd
from phoenix5 import (
    TalonSRX,
    TalonSRXControlMode,
    TalonSRXConfiguration,
    LimitSwitchSource,
    Faults,
    LimitSwitchNormal,
)
from wpilib import SmartDashboard, AnalogInput, RobotBase, Timer

import constants


class Climber(Subsystem):
    CLIMBER_TOP_LIMIT = 5000
    CLIMBER_UP_SPEED = 1
    CLIMBER_DOWN_SPEED = -1

    def __init__(self) -> None:
        super().__init__()

        self._left_climber: TalonSRX = TalonSRX(constants.LEFT_CLIMBER)
        self._left_climber.configFactoryDefault()
        # self._left_climber.configReverseLimitSwitchSource(
        #     LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0
        # )
        # self._left_climber.configForwardSoftLimitThreshold(self.CLIMBER_TOP_LIMIT)
        # self._left_climber.configForwardSoftLimitEnable(True)
        self._left_climber.setSensorPhase(True)
        self._left_climber.setSelectedSensorPosition(0)

        self._right_climber: TalonSRX = TalonSRX(constants.RIGHT_CLIMBER)
        self._right_climber.configFactoryDefault()
        self._right_climber.setInverted(True)
        # self._right_climber.configReverseLimitSwitchSource(
        #     LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0
        # )
        # self._right_climber.configForwardSoftLimitThreshold(self.CLIMBER_TOP_LIMIT)
        # self._right_climber.configForwardSoftLimitEnable(True)

        self._left_faults: Faults = Faults()
        self._right_faults: Faults = Faults()

    def drive_climber_up(self) -> None:
        self._left_climber.set(TalonSRXControlMode.PercentOutput, self.CLIMBER_UP_SPEED)
        self._right_climber.set(
            TalonSRXControlMode.PercentOutput, self.CLIMBER_UP_SPEED
        )

    def drive_climber_down(self) -> None:
        self._left_climber.set(
            TalonSRXControlMode.PercentOutput, self.CLIMBER_DOWN_SPEED
        )
        self._right_climber.set(
            TalonSRXControlMode.PercentOutput, self.CLIMBER_DOWN_SPEED
        )

    def climber_at_top(self) -> bool:
        return (
            self._left_faults.ForwardLimitSwitch
            and self._right_faults.ForwardLimitSwitch
        )

    def stop_climber_motors(self) -> None:
        self._left_climber.set(TalonSRXControlMode.PercentOutput, 0)
        self._right_climber.set(TalonSRXControlMode.PercentOutput, 0)

    def periodic(self) -> None:
        self._left_climber.getFaults(self._left_faults)
        self._right_climber.getFaults(self._right_faults)

        SmartDashboard.putBoolean(
            "L Forward Limit", self._left_faults.ForwardLimitSwitch
        )
        SmartDashboard.putBoolean(
            "R Forward Limit", self._right_faults.ForwardLimitSwitch
        )
        SmartDashboard.putBoolean(
            "L Reverse Limit", self._left_climber.isRevLimitSwitchClosed()
        )
        SmartDashboard.putBoolean(
            "R Reverse Limit", self._right_climber.isRevLimitSwitchClosed()
        )


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
