import wpilib.drive
from commands2 import Subsystem, Command
from phoenix6.configs import TalonFXConfiguration, TalonFXConfigurator
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import (
    InvertedValue,
    NeutralModeValue,
    FeedbackSensorSourceValue,
)
from phoenix6.sim import ChassisReference
from phoenix6.controls import DutyCycleOut, VoltageOut

import constants


class DriveTrain(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._left_leader = TalonFX(constants.DT_LEFT_LEADER)
        self._left_follower = TalonFX(constants.DT_LEFT_FOLLOWER)
        self._right_leader = TalonFX(constants.DT_RIGHT_LEADER)
        self._right_follower = TalonFX(constants.DT_LEFT_FOLLOWER)

        self.__configure_left_side_drive()
        self.__configure_right_side_drive()

        self._left_volts_out: VoltageOut = VoltageOut(0, enable_foc=False)
        self._left_volts_out.update_freq_hz = 0
        self._right_volts_out: VoltageOut = VoltageOut(0, enable_foc=False)
        self._right_volts_out.update_freq_hz = 0

        self._left_percent_out: DutyCycleOut = DutyCycleOut(0, enable_foc=False)
        self._right_percent_out: DutyCycleOut = DutyCycleOut(0, enable_foc=False)

    def __configure_left_side_drive(self) -> None:
        # Applying a new configuration will erase all other config settings since we start with a blank config
        # so each setting needs to be explicitly set here in the config method
        config = TalonFXConfiguration()

        # Set the left side motors to be counter clockwise positive
        config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        # Set the motors to electrically stop instead of coast
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # PID controls will use integrated encoder
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR

        # Apply the configuration to the motors
        self._left_leader.configurator.apply(config)
        self._left_follower.configurator.apply(config)

        self._left_follower.set_control(Follower(self._left_leader.device_id, False))
        self._left_leader.sim_state.Orientation = (
            ChassisReference.CounterClockwise_Positive
        )
        self._left_follower.sim_state.Orientation = (
            ChassisReference.CounterClockwise_Positive
        )

    def __configure_right_side_drive(self) -> None:
        # Applying a new configuration will erase all other config settings since we start with a blank config
        # so each setting needs to be explicitly set here in the config method
        config = TalonFXConfiguration()

        # Set the left side motors to be counter clockwise positive
        config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        # Set the motors to electrically stop instead of coast
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # PID controls will use integrated encoder
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR

        # Apply the configuration to the motors
        self._right_leader.configurator.apply(config)
        self._right_follower.configurator.apply(config)

        self._right_follower.set_control(Follower(self._right_leader.device_id, False))
        self._right_leader.sim_state.Orientation = ChassisReference.Clockwise_Positive
        self._right_follower.sim_state.Orientation = ChassisReference.Clockwise_Positive

    def drive_manually(self, turn: float, forward: float) -> None:
        # self._left_volts_out.output = (forward + turn) * 12.0
        # self._right_volts_out.output = (forward - turn) * 12.0

        # self._left_leader.set_control(self._left_volts_out)
        # self._right_leader.set_control(self._right_volts_out)
        self._left_percent_out.output = forward + turn
        self._right_percent_out.output = forward - turn

        self._left_leader.set_control(self._left_percent_out)
        self._right_leader.set_control(self._right_percent_out)
