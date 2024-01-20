#!/usr/bin/env python3
import math
import wpilib
from wpilib import DriverStation
from commands2 import (
    TimedCommandRobot,
    CommandScheduler,
    Command,
    PrintCommand,
    RunCommand,
    InstantCommand,
    cmd,
)
from commands2.button import CommandXboxController
from wpimath.geometry import Pose2d
from pathplannerlib.auto import (
    NamedCommands,
    PathPlannerAuto,
    AutoBuilder,
    ReplanningConfig,
)
import drivetrain
import constants
from typing import Tuple, List


class MyRobot(TimedCommandRobot):
    """Class that defines the totality of our Robot"""

    def robotInit(self) -> None:
        """
        This method must eventually exit in order to ever have the robot
        code light turn green in DriverStation. So, this will create an
        instance of the Robot that contains all the subsystems,
        button bindings, and operator interface pieces like driver
        dashboards
        """
        # Setup the operator interface (typically CommandXboxController)
        self._driver_controller = CommandXboxController(0)

        # Instantiate any subystems
        self._drivetrain: drivetrain.DriveTrain = drivetrain.DriveTrain()
        wpilib.SmartDashboard.putData("Drivetrain", self._drivetrain)

        self.__configure_default_commands()

        self.__configure_button_bindings()

        self.__configure_autonomous_commands()

        self._auto_command = None

    def __configure_button_bindings(self) -> None:
        self._driver_controller.a().onTrue(
            drivetrain.DriveMMInches(self._drivetrain, 120)
        )
        self._driver_controller.x().onTrue(
            self._drivetrain.configure_turn_pid(90)
            .andThen(self._drivetrain.turn_with_pid())
            .withName("Turn 90")
        )
        self._driver_controller.b().onTrue(
            self._drivetrain.mm_drive_config(45)
            .andThen(self._drivetrain.mm_drive_distance())
            .withName("Drive 45")
        )

    def __configure_default_commands(self) -> None:
        # Setup the default commands for subsystems
        if wpilib.RobotBase.isSimulation():
            # Set the Drivetrain to arcade drive by default
            self._drivetrain.setDefaultCommand(
                # A split-stick arcade command, with forward/backward controlled by the left
                # hand, and turning controlled by the right.
                RunCommand(
                    lambda: self._drivetrain.drive_teleop(
                        -self._driver_controller.getRawAxis(
                            constants.CONTROLLER_FORWARD_SIM
                        ),
                        -self._driver_controller.getRawAxis(
                            constants.CONTROLLER_TURN_SIM
                        ),
                    ),
                    self._drivetrain,
                ).withName("DefaultDrive")
            )
        else:
            self._drivetrain.setDefaultCommand(
                # A split-stick arcade command, with forward/backward controlled by the left
                # hand, and turning controlled by the right.
                RunCommand(
                    lambda: self._drivetrain.drive_teleop(
                        -self._driver_controller.getRawAxis(
                            constants.CONTROLLER_FORWARD_REAL
                        ),
                        self._driver_controller.getRawAxis(
                            constants.CONTROLLER_TURN_REAL
                        ),
                    ),
                    self._drivetrain,
                ).withName("DefaultDrive")
            )

    def __configure_autonomous_commands(self) -> None:
        # Register the named commands used by the PathPlanner auto builder
        # ShootSub
        # DeployIntake
        # FeedShooter
        NamedCommands.registerCommand(
            "ShootSub", PrintCommand("Shoot Note into Speaker")
        )
        NamedCommands.registerCommand("DeployIntake", PrintCommand("DeployIntake"))
        NamedCommands.registerCommand(
            "FeedShooter", PrintCommand("Move Note into SHooter")
        )

        # Configure the builder with an Ramsete trajectory follower
        AutoBuilder.configureRamsete(
            self._drivetrain.get_robot_pose,  # Robot pose supplier
            self._drivetrain.reset_odometry,  # Method to reset odometry (will be called if your auto has a starting pose)
            self._drivetrain.get_wheel_speeds,  # Current ChassisSpeeds supplier
            self._drivetrain.driveSpeeds,  # Method that will drive the robot given ChassisSpeeds
            ReplanningConfig(),  # Default path replanning config. See the API for the options here
            self.should_flip_path,  # Flip if we're on the red side
            self._drivetrain,  # Reference to this subsystem to set requirements
        )

    def getAutonomousCommand(self) -> Command:
        return PathPlannerAuto("TwoRingSub2")

    def teleopInit(self) -> None:
        if self._auto_command is not None:
            self._auto_command.cancel()

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()

    def autonomousInit(self) -> None:
        self._auto_command = self.getAutonomousCommand()

        if self._auto_command is not None:
            self._auto_command.schedule()

    def disabledPeriodic(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        return super().teleopPeriodic()

    def get_starting_pose(self) -> Pose2d:
        return Pose2d(1.34, 5.55, math.pi)

    def should_flip_path(self) -> bool:
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
