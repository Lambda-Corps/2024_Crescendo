import math

import wpilib.drive
from wpilib import RobotBase, DriverStation
import wpilib.simulation
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    DifferentialDriveOdometry,
    DifferentialDriveKinematics,
    ChassisSpeeds,
    DifferentialDriveWheelSpeeds,
)
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, Trajectory
from wpilib import SmartDashboard, Field2d
from commands2 import Subsystem, Command, cmd, InstantCommand
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXConfigurator,
    MotionMagicConfigs,
    Slot0Configs,
)
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import (
    InvertedValue,
    NeutralModeValue,
    FeedbackSensorSourceValue,
    MotionMagicIsRunningValue,
    ControlModeValue,
)
from phoenix6.sim import ChassisReference
from phoenix6.controls import (
    DutyCycleOut,
    VoltageOut,
    MotionMagicVoltage,
)
from phoenix6.unmanaged import feed_enable
import navx
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.commands import FollowPathRamsete
from pathplannerlib.config import ReplanningConfig, PIDConstants

import constants


class DriveTrain(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self._gyro: navx.AHRS = navx.AHRS.create_spi()
        SmartDashboard.putData("Navx", self._gyro)

        # Create the output objects for the talons, currently one each for
        # the following modes: VoltageOut, PercentOutput, and MotionMagic
        self.__create_output_objects()

        # Create the PID controller setup for turning
        self.__create_turn_pid_objects()

        # Apply all the configurations to the left and right side Talons
        self.__configure_left_side_drive()
        self.__configure_right_side_drive()

        # Create the Odometry tracker
        self._odometry: DifferentialDriveOdometry = DifferentialDriveOdometry(
            self._gyro.getRotation2d(), 0, 0
        )

        self._kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(
            constants.DT_TRACKWIDTH_METERS
        )

        if RobotBase.isSimulation():
            self.__configure_simulation()

        self._field = Field2d()
        SmartDashboard.putData("MyField", self._field)

    def __configure_simulation(self) -> None:
        self._sim_gyro = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self._sim_gyro.getDouble("Yaw")
        self.navx_comp = self._sim_gyro.getDouble("CompassHeading")

        self._system = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3)
        self._drivesim = wpilib.simulation.DifferentialDrivetrainSim(
            self._system,
            constants.DT_TRACKWIDTH_METERS,
            DCMotor.falcon500(4),
            constants.DT_GEAR_RATIO,
            constants.DT_WHEEL_RADIUS_INCHES,
        )

        self._left_leader.sim_state.orientation = ChassisReference.Clockwise_Positive
        self._right_leader.sim_state.orientation = (
            ChassisReference.CounterClockwise_Positive
        )

    def __configure_motion_magic(self, config: TalonFXConfiguration) -> None:
        self._mm_setpoint = 0

        self._mm_tolerance = (
            math.pi * constants.DT_WHEEL_DIAMETER
        ) / 50  # end within 1/50th of a rotation
        config.motion_magic.motion_magic_cruise_velocity = (
            10  # 1 rps = 1.5 fps, 10 rps = 15 fps
        )
        config.motion_magic.motion_magic_acceleration = (
            20  # .5 seconds to reach full speed
        )

        # Motion Magic slot will be 0
        slot_0: Slot0Configs = config.slot0
        if RobotBase.isSimulation():
            slot_0.k_p = 1.2  # 1 full wheel rotation will correct with 1.2v power
            slot_0.k_s = 0.01  # 1 percent maybe accounts for friction?
        else:
            # TODO -- Tune these on the robot
            slot_0.k_p = 12  # 1 full wheel rotation will correct with 12 volts
            slot_0.k_s = 0

    def __create_output_objects(self) -> None:
        self._left_volts_out: VoltageOut = VoltageOut(0, enable_foc=False)
        # self._left_volts_out.update_freq_hz = 0
        self._right_volts_out: VoltageOut = VoltageOut(0, enable_foc=False)
        # self._right_volts_out.update_freq_hz = 0

        self._left_percent_out: DutyCycleOut = DutyCycleOut(0, enable_foc=False)
        self._right_percent_out: DutyCycleOut = DutyCycleOut(0, enable_foc=False)

        self._mm_out: MotionMagicVoltage = MotionMagicVoltage(0, enable_foc=False)

    def __create_turn_pid_objects(self) -> None:
        self._turn_setpoint = 0
        self._turn_tolerance = 0.5  # within 3 degrees we'll call good enough
        if RobotBase.isSimulation():
            self._turn_pid_controller: PIDController = PIDController(0.00175, 0.0, 0.0)
            self._turn_kF = 0.049
        else:
            # These must be tuned
            self._turn_pid_controller: PIDController = PIDController(0, 0, 0)
            self._turn_kF = 0.1  # TODO Tune me

    def __configure_left_side_drive(self) -> None:
        self._left_leader = TalonFX(constants.DT_LEFT_LEADER)
        # self._left_follower = TalonFX(constants.DT_LEFT_FOLLOWER)
        # Applying a new configuration will erase all other config settings since we start with a blank config
        # so each setting needs to be explicitly set here in the config method
        config = TalonFXConfiguration()

        # Set the left side motors to be counter clockwise positive
        config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        # Set the motors to electrically stop instead of coast
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # PID controls will use integrated encoder
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR

        config.feedback.sensor_to_mechanism_ratio = constants.DT_GEAR_RATIO

        # Configure the motion magic objects and store them as member variables
        self.__configure_motion_magic(config)

        # Apply the configuration to the motors
        self._left_leader.configurator.apply(config)
        # self._left_follower.configurator.apply(config)

        # self._left_follower.set_control(Follower(self._left_leader.device_id, False))
        self._left_leader.sim_state.Orientation = ChassisReference.Clockwise_Positive
        # self._left_follower.sim_state.Orientation = (
        #     ChassisReference.Clockwise_Positive
        # )

    def __configure_right_side_drive(self) -> None:
        self._right_leader = TalonFX(constants.DT_RIGHT_LEADER)
        # self._right_follower = TalonFX(constants.DT_LEFT_FOLLOWER)
        # Applying a new configuration will erase all other config settings since we start with a blank config
        # so each setting needs to be explicitly set here in the config method
        config = TalonFXConfiguration()

        # Set the left side motors to be counter clockwise positive
        config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        # Set the motors to electrically stop instead of coast
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # PID controls will use integrated encoder
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR

        # Configure the motion magic objects and store them as member variables
        self.__configure_motion_magic(config)

        config.feedback.sensor_to_mechanism_ratio = constants.DT_GEAR_RATIO
        # Apply the configuration to the motors
        self._right_leader.configurator.apply(config)
        # self._right_follower.configurator.apply(config)

        # self._right_follower.set_control(Follower(self._right_leader.device_id, False))
        self._right_leader.sim_state.Orientation = (
            ChassisReference.CounterClockwise_Positive
        )
        # self._right_follower.sim_state.Orientation = ChassisReference.CounterClockwise_Positive

    def drive_teleop(self, forward: float, turn: float) -> None:
        turn = self.__deadband(turn, 0.05)
        forward = self.__deadband(forward, 0.05)

        speeds = wpilib.drive.DifferentialDrive.curvatureDriveIK(forward, turn, True)

        self._left_volts_out.output = speeds.left * 12.0
        self._right_volts_out.output = speeds.right * 12.0

        self._left_leader.set_control(self._left_volts_out)
        self._right_leader.set_control(self._right_volts_out)

    def drive_percent_out(self, forward: float, turn: float) -> None:
        turn = self.__deadband(turn, 0.05)
        forward = self.__deadband(forward, 0.05)

        speeds = wpilib.drive.DifferentialDrive.curvatureDriveIK(forward, turn, True)

        self._left_percent_out.output = speeds.left
        self._right_percent_out.output = speeds.right

        self._left_leader.set_control(self._left_percent_out)
        self._right_leader.set_control(self._right_percent_out)

    def drive_volts(self, left: float, right: float) -> None:
        self._left_volts_out.output = left
        self._right_volts_out.output = right

        SmartDashboard.putNumber("LeftVolts", self._left_volts_out.output)
        SmartDashboard.putNumber("RightVolts", self._right_volts_out.output)
        self._left_leader.set_control(self._left_volts_out)
        self._right_leader.set_control(self._right_volts_out)

    def driveSpeeds(self, speeds: ChassisSpeeds) -> None:
        speeds: DifferentialDriveWheelSpeeds = self._kinematics.toWheelSpeeds(speeds)
        # if RobotBase.isSimulation():
        #     speeds.left *= -1
        self.drive_volts(speeds.left, speeds.right)

    def __deadband(self, input: float, abs_min: float) -> float:
        """ """
        if abs_min < 0:
            abs_min *= -1

        if input < 0 and input > abs_min * -1:
            input = 0

        if input > 0 and input < abs_min:
            input = 0

        return input

    def periodic(self) -> None:
        SmartDashboard.putNumber("LeftEncoder", self._left_leader.get_position().value)
        SmartDashboard.putNumber(
            "RightEncoder", self._right_leader.get_position().value
        )

        pose = self._odometry.update(
            Rotation2d().fromDegrees(self.__get_gyro_heading()),
            self.__rotations_to_meters(
                self._left_leader.get_position().value_as_double
            ),
            self.__rotations_to_meters(
                self._right_leader.get_position().value_as_double
            ),
        )

        self._field.setRobotPose(pose)

    def simulationPeriodic(self) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.
        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # Currently, the Python API for CTRE doesn't automatically detect the the
        # Sim driverstation status and enable the signals. So, for now, manually
        # feed the enable signal for double the set robot period.
        feed_enable(constants.ROBOT_PERIOD_MS * 2)

        # Start the motor simulation work flow by passing robot battery voltage to sim motors
        self._left_leader.sim_state.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        self._right_leader.sim_state.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        # self._l_follow_motor.set_supply_voltage(
        #     wpilib.RobotController.getBatteryVoltage()
        # )
        # self._r_follow_motor.set_supply_voltage(
        #     wpilib.RobotController.getBatteryVoltage()
        # )

        # Apply the motor inputs to the simulation
        self._drivesim.setInputs(
            self._left_leader.sim_state.motor_voltage,
            self._right_leader.sim_state.motor_voltage,
        )

        # advance the simulation model a timing loop
        self._drivesim.update(constants.ROBOT_PERIOD_MS)

        # Update the motor values with the new calculated values from the physics engine
        self._left_leader.sim_state.set_raw_rotor_position(
            self.__feet_to_encoder_rotations(self._drivesim.getLeftPositionFeet())
        )
        self._left_leader.sim_state.set_rotor_velocity(
            self.__velocity_feet_to_rps(self._drivesim.getLeftVelocityFps())
        )
        self._right_leader.sim_state.set_raw_rotor_position(
            self.__feet_to_encoder_rotations(self._drivesim.getRightPositionFeet())
        )
        self._right_leader.sim_state.set_rotor_velocity(
            self.__velocity_feet_to_rps(self._drivesim.getRightVelocityFps())
        )
        # self._l_follow_motor.set_raw_rotor_position(
        #     self.__feet_to_encoder_ticks(self._drivesim.getLeftPositionFeet())
        # )
        # self._l_follow_motor.set_rotor_velocity(
        #     self.__velocity_feet_to_talon_ticks(self._drivesim.getLeftVelocityFps())
        # )
        # self._r_follow_motor.set_raw_rotor_position(
        #     self.__feet_to_encoder_ticks(self._drivesim.getRightPositionFeet())
        # )
        # self._r_follow_motor.set_rotor_velocity(
        #     self.__velocity_feet_to_talon_ticks(self._drivesim.getRightVelocityFps())
        # )

        # Update the gyro simulation
        degrees = self._drivesim.getHeading().degrees()
        self.navx_yaw.set(degrees)
        self.navx_comp.set(degrees)

    def configure_motion_magic(self, distance_in_inches: float) -> None:
        """
        Method to configure the motors using a MotionMagic profile.

        :param: distance_in_inches  Distance in inches to travel.
        """
        # The TalonFX configuration already sets up the SensorToMechanism ratio when
        # it accounts for the position feedback.  So, when the talon.get_position()
        # method is called, it returns the roations of the wheel's output shaft, not
        # the input shaft of the gearbox.  So, if we were to use a setpoint of 1 (1 rotation)
        # in the MotionMagic profile, it would try to turn the wheels 1 rotation, not
        # the motor.

        # Convert the distance in inches, to wheel rotations
        #                          distance_in_inches
        # wheel_rotations =   --------------------------
        #                        Pi * Wheel Diameter
        distance_in_rotations = distance_in_inches / (
            math.pi * constants.DT_WHEEL_DIAMETER
        )
        curr_right = self._right_leader.get_position().value_as_double

        self._mm_out.with_position(curr_right + distance_in_rotations).with_slot(0)

    def drive_motion_magic(self) -> None:
        self._left_leader.set_control(Follower(self._right_leader.device_id, False))
        self._right_leader.set_control(self._mm_out)

    def at_mm_setpoint(self) -> bool:
        curr_right = self._right_leader.get_position().value

        return abs(self._mm_out.position - curr_right) < self._mm_tolerance

    def mm_drive_distance(self) -> Command:
        return (
            cmd.run(lambda: self.drive_motion_magic(), self)
            .until(lambda: self.at_mm_setpoint())
            .withName("DriveMM")
        )

    def mm_drive_config(self, distance_in_inches: float) -> Command:
        return cmd.runOnce(lambda: self.configure_motion_magic(distance_in_inches))

    def configure_turn_pid(self, desired_angle: float) -> Command:
        return cmd.runOnce(lambda: self.__config_turn_command(desired_angle))

    def turn_with_pid(self) -> Command:
        return (
            cmd.run(lambda: self.__turn_with_pid(), self)
            .until(lambda: self.__at_turn_setpoint())
            .withName("TurnWithPID")
        )

    def __config_turn_command(self, desired_angle: float) -> None:
        self._turn_setpoint = desired_angle + self._gyro.getYaw()
        self._turn_pid_controller.setSetpoint(self._turn_setpoint)
        self._turn_pid_controller.setTolerance(self._turn_tolerance)
        wpilib.SmartDashboard.putNumber("Turn Setpoint", self._turn_setpoint)

    def __turn_with_pid(self) -> None:
        curr_angle = self._gyro.getYaw()
        wpilib.SmartDashboard.putNumber("Yaw", curr_angle)

        pidoutput = self._turn_pid_controller.calculate(curr_angle)

        # Promote the value to at least the KF
        if (pidoutput < 0) and (pidoutput > -self._turn_kF):
            pidoutput = -self._turn_kF
        elif (pidoutput > 0) and (pidoutput < self._turn_kF):
            pidoutput = self._turn_kF

        wpilib.SmartDashboard.putNumber("PID Out", pidoutput)

        if RobotBase.isSimulation():
            self.drive_teleop(pidoutput, 0)
        else:
            self.drive_teleop(0, pidoutput)

    def __at_turn_setpoint(self) -> bool:
        curr_angle = self._gyro.getYaw()

        wpilib.SmartDashboard.putBoolean(
            "At Setpoint", self._turn_pid_controller.atSetpoint()
        )

        return abs(curr_angle - self._turn_setpoint) < self._turn_tolerance

    def __get_gyro_heading(self) -> float:
        return self._gyro.getAngle()

    def get_robot_pose(self) -> Pose2d:
        return self._odometry.getPose()
        # if RobotBase.isSimulation():
        #     return self._drivesim.getPose()
        # else:
        #     return self._odometry.getPose()

    def get_wheel_speeds(self) -> ChassisSpeeds:
        diff_speed: DifferentialDriveWheelSpeeds = DifferentialDriveWheelSpeeds(
            self.__rps_to_mps(self._left_leader.get_velocity().value_as_double),
            self.__rps_to_mps(self._right_leader.get_velocity().value_as_double),
        )
        return self._kinematics.toChassisSpeeds(diff_speed)

    def __rps_to_mps(self, rotations: float) -> float:
        return rotations * (math.pi * constants.DT_WHEEL_DIAMETER)

    def reset_odometry(self, pose: Pose2d) -> None:
        self._gyro.setAngleAdjustment(pose.rotation().degrees())
        rot2d: Rotation2d = Rotation2d.fromDegrees(pose.rotation().degrees())
        self._odometry.resetPosition(rot2d, 0, 0, pose)
        # if RobotBase.isSimulation():
        #     self._drivesim.setPose(pose)

    def reset_angle_offset(self) -> None:
        self._gyro.setAngleAdjustment(0)

    def __feet_to_encoder_rotations(self, distance_in_feet: float) -> float:
        #                    feet * 12
        # rotations = ---------------------  * gear ratio
        #             2pi * wheel_diameter
        wheel_rotations = (distance_in_feet * 12) / (
            math.pi * constants.DT_WHEEL_DIAMETER
        )
        motor_rotations = wheel_rotations * constants.DT_GEAR_RATIO
        return motor_rotations

    def __velocity_feet_to_rps(self, velocity_in_feet: float) -> float:
        #             velocity * 12
        # rps = -------------------------- * gear ratio
        #          2pi * wheel diameter
        wheel_rotations_per_second = (velocity_in_feet * 12) / (
            math.pi * constants.DT_WHEEL_DIAMETER
        )
        motor_rotations_per_second = (
            wheel_rotations_per_second * constants.DT_GEAR_RATIO
        )

        return motor_rotations_per_second

    def __rotations_to_meters(self, rotations: float) -> float:
        return rotations * constants.DT_WHEEL_CIRCUMFERENCE_METERS

    def follow_path_command(self, pathname: str) -> Command:
        path: PathPlannerPath = PathPlannerPath.fromPathFile(pathname)
        ramsete_cmd = FollowPathRamsete(
            path,
            self.get_robot_pose,  # Robot pose supplier
            self.get_wheel_speeds,  # Current ChassisSpeeds supplier
            self.driveSpeeds,  # Method that will drive the robot given ChassisSpeeds
            ReplanningConfig(),  # Default path replanning config. See the API for the options here
            self.should_flip_path,  # Flip if we're on the red side
            self,  # this drivetrain (for requirements)
        )

        return (
            cmd.runOnce(lambda: self.reset_odometry(path.getStartingDifferentialPose()))
            .andThen(ramsete_cmd)
            .andThen(cmd.runOnce(lambda: self.drive_volts(0, 0)))
        )

    def should_flip_path(self) -> bool:
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed


class DriveMMInches(Command):
    def __init__(self, dt: DriveTrain, distance_in_inches: float) -> None:
        super().__init__()

        self._dt = dt
        self._desired_distance = distance_in_inches

        # Tell the scheduler this requires the drivetrain
        self.addRequirements(self._dt)

    def initialize(self):
        self._dt.configure_motion_magic(self._desired_distance)

    def execute(self):
        self._dt.drive_motion_magic()

    def isFinished(self) -> bool:
        return self._dt.at_mm_setpoint()

    def end(self, interrupted: bool):
        self._dt.drive_volts(0, 0)
