#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units

from wpimath.system.plant import DCMotor, LinearSystemId
import math
import typing
import phoenix6
from phoenix6.unmanaged import feed_enable
import constants


if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller

        # Keep a reference to the robot passed in
        self._robot_instance = robot

        # Create a sim Gyro to be used for maintaining the heading
        # self._gyro = wpilib.simulation.AnalogGyroSim(robot._gyro)
        self._gyro = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self._gyro.getDouble("Yaw")

        self.position = 0

        # Change these parameters to fit your robot!
        bumper_width = 3.25
        # fmt: off
        self._system = LinearSystemId.identifyDrivetrainSystem(1.98, .2, 1.5, .3)
        self._drivesim = wpilib.simulation.DifferentialDrivetrainSim(
            self._system,
            constants.DT_TRACKWIDTH_METERS,
            DCMotor.falcon500(4),
            constants.DT_GEAR_RATIO,
            constants.DT_WHEEL_RADIUS_INCHES,
        )
        # fmt: on
        self._l_lead_motor: phoenix6.sim.TalonFXSimState = (
            robot._drivetrain._left_leader.sim_state
        )
        # self._l_follow_motor: phoenix6.sim.TalonFXSimState = (
        #     robot._drivetrain._left_follower.sim_state
        # )
        self._r_lead_motor: phoenix6.sim.TalonFXSimState = (
            robot._drivetrain._right_leader.sim_state
        )
        # self._r_follow_motor: phoenix6.sim.TalonFXSimState = (
        #     robot._drivetrain._right_follower.sim_state
        # )

    def update_sim(self, now: float, tm_diff: float) -> None:
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

        # CTRE simulation is low-level, it ignores some of the things like motor
        # motor invresion etc.  WPILib wants +v to be forward.
        # Start the motor simulation work flow by passing robot battery voltage to sim motors
        self._l_lead_motor.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        self._r_lead_motor.set_supply_voltage(
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
            self._l_lead_motor.motor_voltage,
            self._r_lead_motor.motor_voltage,
        )

        # advance the simulation model a timing loop
        self._drivesim.update(tm_diff)

        # transform = self.drivetrain.calculate(speeds[self.LEFT_SPEED_INDEX], speeds[self.RIGHT_SPEED_INDEX], tm_diff)
        # pose = self.physics_controller.move_robot(transform)
        self._l_lead_motor.set_raw_rotor_position(
            -self.__feet_to_encoder_rotations(self._drivesim.getLeftPositionFeet())
        )
        self._l_lead_motor.set_rotor_velocity(
            -self.__velocity_feet_to_rps(self._drivesim.getLeftVelocityFps())
        )
        self._r_lead_motor.set_raw_rotor_position(
            self.__feet_to_encoder_rotations(self._drivesim.getRightPositionFeet())
        )
        self._r_lead_motor.set_rotor_velocity(
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
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        pose = self._drivesim.getPose()
        self.navx_yaw.set(self._drivesim.getHeading().degrees())
        # self.navx_yaw.set(-pose.rotation().degrees())

        self.physics_controller.field.setRobotPose(pose)

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

    def __meters_to_encoder_ticks(self, distance_in_meters: float) -> int:
        return int((distance_in_meters * constants.DT_TICKS_PER_METER))

    def __velocity_meters_to_talon_ticks(self, velocity_in_meters: float) -> int:
        wheel_rotations_per_second = velocity_in_meters / (
            2 * math.pi * constants.DT_WHEEL_DIAMETER
        )
        wheel_rotations_per_100ms = (
            wheel_rotations_per_second * constants.DT_GEAR_RATIO
        ) / 10
        motor_rotations_per_100ms = wheel_rotations_per_100ms * constants.DT_GEAR_RATIO
        return int(motor_rotations_per_100ms * constants.DT_TICKS_PER_INCH)
