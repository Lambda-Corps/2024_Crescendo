from enum import Enum
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXConfigurator,
)
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue
from phoenix6.controls import DutyCycleOut, VelocityVoltage
from phoenix6.unmanaged import feed_enable
from phoenix5 import TalonSRX, TalonSRXConfiguration, ControlMode

import constants

SPEAKER_RPS = 37
FLYWHEEL_SPEEED = 0.6
SHOOTER_MIN = 0.322
SPEAKER_FROM_RING2 = 0.330
AMP_FROM_AMP = 0.348
SPEAKER_FROM_SUB = 0.353

RPS = 0
PERCENT_OUT = 1
LOCATION = 2


class ShooterPosition(Enum):
    """
    Enum holding meta data information for the various
    shooting locations on the field.

    [ Top Flywheel RPS, Bottom Flywheel % output, Shooter Ramp Angle]
    """

    SUBWOOFER_1 = [52, 1.0, SPEAKER_FROM_SUB]
    SUBWOOFER_2 = [52, 1.0, SPEAKER_FROM_SUB]
    SUBWOOFER_3 = [52, 1.0, SPEAKER_FROM_SUB]
    RING_1 = [58, 1.0, SPEAKER_FROM_RING2]
    RING_2 = [58, 1.0, SPEAKER_FROM_RING2]
    RING_3 = [58, 1.0, SPEAKER_FROM_RING2]
    AMP = [25, 0.5, AMP_FROM_AMP]
    MIN = [58, 1.0, SHOOTER_MIN]

class Shooter(Subsystem):
    """
    Test class for shooter prototype
    """

    def __init__(self, test_mode=False):
        super().__init__()
        self._shooter_left: TalonFX = self.__configure_left_side()
        self._shooter_right: TalonFX = self.__configure_right_side()
        self._shooter_ramp: TalonSRX = self.__configure_shooter_ramp()
        self._shooter_ramp_angle: DutyCycleEncoder = self.__configure_ramp_encoder()

        self.__test_mode = test_mode
        if self.__test_mode:
            SmartDashboard.putNumber("ShooterRPS", SPEAKER_RPS)
            SmartDashboard.putNumber("ShooterPercent", FLYWHEEL_SPEEED)

        # For now use DutyCycle, but should configure for MotionMagicVelocity
        # later on.
        # self._motor_output = DutyCycleOut(0, enable_foc=False)
        self._motor_output = VelocityVoltage(0, enable_foc=False, slot=0)

        self._motor_rps = SPEAKER_RPS

        if RobotBase.isSimulation():
            self._shooter_sim: FlywheelSim = FlywheelSim(
                DCMotor.falcon500(1), constants.SHOOTER_GEARING, constants.SHOOTER_MOI
            )

            self._sim_counter = 0

        self._angle = 0

        # We should be starting at the subwoofer
        self._curr_location = ShooterPosition.SUBWOOFER_2

    def __configure_left_side(
        self,
    ) -> TalonFX:
        talon = TalonFX(constants.FLYWHEEL_LEFT)

        # Perform any other configuration
        config: TalonFXConfiguration = TalonFXConfiguration()
        config.slot0.k_s = 0.15  # Measured .15 volts to overcome static friction
        config.slot0.k_v = 0.11  # Measured .19 for 1ps
        config.slot0.k_a = 0.01  # complete guess, not measured
        config.slot0.k_p = (
            0.18  # an error of 1 RPS should add almost .19 more to correct
        )
        config.slot0.k_i = 0
        config.slot0.k_d = 0

        config.motor_output.neutral_mode = NeutralModeValue.COAST

        # Set neutral mode to coast
        talon.configurator.apply(config)

        # Setup the second set of flywheels with the SRX motors
        self._lowerleft = TalonSRX(constants.INDEX_LEFT)
        self._lowerleft.configFactoryDefault()

        return talon

    def __configure_right_side(
        self,
    ) -> TalonFX:
        # Motors are CCW positive, so the shooter needs the right side
        # to spin CW in order to shoot note.  The left side is already in the
        # correct orientation, so for our right side configuration just needs to
        # follow, but oppose the leader and it will give us the spin we want.
        talon = TalonFX(constants.FLYWHEEL_RIGHT)
        config: TalonFXConfiguration = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        talon.configurator.apply(config)

        talon.set_control(
            Follower(constants.FLYWHEEL_LEFT, oppose_master_direction=True)
        )

        # Setup the second set of flywheels
        self.lowerright = TalonSRX(constants.INDEX_RIGHT)
        self.lowerright.configFactoryDefault()

        return talon

    def __configure_shooter_ramp(self) -> TalonSRX:
        talon: TalonSRX = TalonSRX(constants.SHOOTER_ELEVATOR)
        talon.configFactoryDefault()
        # Measure what to do here, make any other configuration adjustments
        talon.setInverted(False)

        return talon

    def __configure_ramp_encoder(self) -> DutyCycleEncoder:
        encoder: DutyCycleEncoder = DutyCycleEncoder(constants.SHOOTER_ANGLE_ENCODER)

        return encoder

    def drive_motors(self):

        if self.__test_mode:
            self._motor_output.velocity = SmartDashboard.getNumber(
                "ShooterRPS", SPEAKER_RPS
            )
            speed_775 = SmartDashboard.getNumber("ShooterPercent", FLYWHEEL_SPEEED)
        else:
            speed_775: float = self._curr_location.value[PERCENT_OUT]
            self._motor_output.velocity = self._curr_location.value[RPS]
        self._shooter_left.set_control(self._motor_output)
        self._lowerleft.set(ControlMode.PercentOutput, speed_775)
        self.lowerright.set(ControlMode.PercentOutput, speed_775)

    def stop_motors(self) -> None:
        self._motor_output.velocity = 0
        self._shooter_left.set_control(self._motor_output)
        self._lowerleft.set(ControlMode.PercentOutput, 0)
        self.lowerright.set(ControlMode.PercentOutput, 0)

    def periodic(self) -> None:
        if self.__test_mode:
            SmartDashboard.putNumber(
                "Velocity", self._shooter_left.get_velocity().value_as_double
            )

        if self._shooter_ramp_angle.isConnected():
            SmartDashboard.putNumber(
                "Encoder Pos", self._shooter_ramp_angle.getAbsolutePosition()
            )

    def shooter_at_speed(self) -> bool:
        # For some reason the simulator speed is broken, so if we're in the simulator
        # Just return after some approximation of 1 second
        if RobotBase.isSimulation():
            self._sim_counter += 1
            if self._sim_counter % 50:
                return True
        else:
            # In the real robot, return the actual velocity
            return (
                self._shooter_left.get_velocity().value_as_double >= self._motor_rps - 1
            )

    def set_shooter_speed(self, speed: float) -> None:

        self._motor_rps = SmartDashboard.getNumber("ShooterRPS", 0)

    def drive_shooter_ramp(self, speed: float) -> None:
        self._shooter_ramp.set(ControlMode.PercentOutput, speed)

    def set_shooter_location(self, location: ShooterPosition) -> None:
        self._curr_location = location

    def move_ramp_to_location(self) -> None:
        speed = 0
        curr_location = self._shooter_ramp_angle.getAbsolutePosition()
        if curr_location < (self._curr_location.value[LOCATION] - 0.001):
            # Ramp needs to move up
            speed = 1
        elif curr_location > (self._curr_location.value[LOCATION] - 0.001):
            # Ramp needs to move down
            speed = -1
        else:
            speed = 0
        self._shooter_ramp.set(ControlMode.PercentOutput, speed)

    def shooter_at_angle(self) -> bool:
        curr_diff = (
            self._shooter_ramp_angle.getAbsolutePosition()
            - self._curr_location.value[LOCATION]
        )
        SmartDashboard.putNumber("ShooterDiff", curr_diff)
        return abs(curr_diff) < 0.0015

    def stop_shooter_ramp(self) -> None:
        self._shooter_ramp.set(ControlMode.PercentOutput, 0)

    def simulationPeriodic(self) -> None:
        feed_enable(constants.ROBOT_PERIOD_MS * 2)

        # Start the motor simulation work flow by passing robot battery voltage to sim motors
        self._shooter_left.sim_state.set_supply_voltage(
            RobotController.getBatteryVoltage()
        )
        self._shooter_right.sim_state.set_supply_voltage(
            RobotController.getBatteryVoltage()
        )

        # Apply the motor inputs to the simulation
        self._shooter_sim.setInput([self._shooter_left.sim_state.motor_voltage])

        # advance the simulation model a timing loop
        self._shooter_sim.update(constants.ROBOT_PERIOD_MS)

        # Update the motor values with the new calculated values from the physics engine
        self._shooter_left.sim_state.set_rotor_velocity(
            self.__radianspersec_to_rotationspersec(
                self._shooter_sim.getAngularVelocity()
            )
        )

    def __radianspersec_to_rotationspersec(self, rad_per_sec: float) -> float:
        # One radian per second equates to 0.0159154943, use that
        return rad_per_sec * 0.159154943


class SetShooter(Command):
    def __init__(self, shooter: Shooter, location: ShooterPosition):
        self._location = location

        self._shooter = shooter

        self.addRequirements(self._shooter)

    def initialize(self):
        self._shooter.set_shooter_location(self._location)

    def execute(self):
        self._shooter.move_ramp_to_location()

    def isFinished(self) -> bool:
        return self._shooter.shooter_at_angle()

    def end(self, interrupted: bool):
        self._shooter.stop_shooter_ramp()
