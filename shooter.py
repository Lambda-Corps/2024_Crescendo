from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXConfigurator,
)
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue
from phoenix6.controls import DutyCycleOut, VelocityVoltage

import constants


class Shooter(Subsystem):
    """
    Test class for shooter prototype
    """

    SPEAKER_RPS = 52  # measured with Tuner, putting motors at .55 (55%)

    def __init__(self):
        super().__init__()
        self._shooter_left: TalonFX = self.__configure_left_side()
        self._shooter_right: TalonFX = self.__configure_right_side()

        SmartDashboard.putNumber("ShooterRPS", self.SPEAKER_RPS)

        # For now use DutyCycle, but should configure for MotionMagicVelocity
        # later on.
        # self._motor_output = DutyCycleOut(0, enable_foc=False)
        self._motor_output = VelocityVoltage(0, enable_foc=False)

        self._motor_rps = self.SPEAKER_RPS

    def __configure_left_side(
        self,
    ) -> TalonFX:
        talon = TalonFX(constants.FLYWHEEL_LEFT)

        # Perform any other configuration
        config: TalonFXConfiguration = TalonFXConfiguration()
        config.slot0.k_s = 0.15  # Measured .15 volts to overcome static friction
        config.slot0.k_v = 0.19  # Measured .19 for 1ps
        config.slot0.k_a = 0.01  # complete guess, not measured
        config.slot0.k_p = (
            0.18  # an error of 1 RPS should add almost .19 more to correct
        )
        config.slot0.k_i = 0
        config.slot0.k_d = 0

        # Set neutral mode to coast
        talon.configurator.apply(config)

        return talon

    def __configure_right_side(
        self,
    ) -> TalonFX:
        # Motors are CCW positive, so the shooter needs the right side
        # to spin CW in order to shoot note.  The left side is already in the
        # correct orientation, so for our right side configuration just needs to
        # follow, but oppose the leader and it will give us the spin we want.
        talon = TalonFX(constants.FLYWHEEL_RIGHT)

        # Neutral mode to coast

        talon.set_control(
            Follower(constants.FLYWHEEL_LEFT, oppose_master_direction=True)
        )

        return talon

    def drive_motors(self):
        self._motor_output.velocity = self._motor_rps
        self._shooter_left.set_control(self._motor_output)

    def periodic(self) -> None:
        SmartDashboard.putNumber(
            "Velocity", self._shooter_left.get_velocity().value_as_double
        )

    def shooter_at_speed(self) -> bool:
        return self._shooter_left.get_velocity().value_as_double >= self._motor_rps - 1

    def set_shooter_speed(self, speed: float) -> None:
        self._motor_rps = speed

    def run_shooter(self) -> Command:
        return RunCommand(lambda: self.drive_motors(), self)


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
        self._shootspeed = SmartDashboard.getNumber("ShooterRPS", 0)
        self._sub.set_shooter_speed(self._shootspeed)

    def execute(self):
        self._sub.drive_motors()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._sub.drive_motors(0)
