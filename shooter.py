from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXConfigurator,
)
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue
from phoenix6.controls import DutyCycleOut

import constants


class Shooter(Subsystem):
    """
    Test class for shooter prototype
    """

    def __init__(self):
        super().__init__()
        self._shooter_left: TalonFX = self.__configure_left_side()
        self._shooter_right: TalonFX = self.__configure_right_side()

        SmartDashboard.putNumber("ShooterSpeed", 0)

        # For now use DutyCycle, but should configure for MotionMagicVelocity
        # later on.
        self._motor_output = DutyCycleOut(0, enable_foc=False)

    def __configure_left_side(
        self,
    ) -> TalonFX:
        talon = TalonFX(constants.FLYWHEEL_LEFT)

        # Perform any other configuration

        return talon

    def __configure_right_side(
        self,
    ) -> TalonFX:
        # Motors are CCW positive, so the shooter needs the right side
        # to spin CW in order to shoot note.  The left side is already in the
        # correct orientation, so for our right side configuration just needs to
        # follow, but oppose the leader and it will give us the spin we want.
        talon = TalonFX(constants.FLYWHEEL_RIGHT)
        talon.set_control(
            Follower(constants.FLYWHEEL_LEFT, oppose_master_direction=True)
        )

        return talon

    def drive_motors(self, speed: float):
        self._motor_output.output = speed
        self._shooter_left.set_control(self._motor_output)



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
        SmartDashboard.putString("speed", f"Got Speed: {self._shootspeed}")

    def execute(self):
        self._sub.drive_motors(self._shootspeed)
        SmartDashboard.putString("executing", f"execute: {self._shootspeed}")

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._sub.drive_motors(0)
