#!/usr/bin/env python3
import math
import wpilib
from wpilib import RobotBase
from commands2 import (
    TimedCommandRobot,
    CommandScheduler,
    Command,
    PrintCommand,
    RunCommand,
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
        self._current_pose = Pose2d()

    def __configure_button_bindings(self) -> None:
        self._driver_controller.a().onTrue(
            self._drivetrain.follow_path_command("SubRing2")
        )
        self._driver_controller.b().onTrue(
            self._drivetrain.follow_path_command("SubRing1")
        )
        self._driver_controller.x().onTrue(
            self._drivetrain.follow_path_command("Ring1Sub")
        )
        self._driver_controller.y().onTrue(
            self._drivetrain.follow_path_command("TestLong")
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

        AutoBuilder.configureRamsete(
            self._drivetrain.get_robot_pose,  # Robot pose supplier
            self._drivetrain.reset_odometry,  # Method to reset odometry (will be called if your auto has a starting pose)
            self._drivetrain.get_wheel_speeds,  # Current ChassisSpeeds supplier
            self._drivetrain.driveSpeeds,  # Method that will drive the robot given ChassisSpeeds
            ReplanningConfig(),  # Default path replanning config. See the API for the options here
            self._drivetrain.should_flip_path,  # Flip if we're on the red side
            self._drivetrain,  # Reference to this subsystem to set requirements
        )

    def getAutonomousCommand(self) -> Command:
        return PathPlannerAuto("Test")

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

    def save_pose(self, pose: Pose2d) -> None:
        if RobotBase.isSimulation():
            self._current_pose = pose

    def get_saved_pose(self) -> Pose2d:
        return self._current_pose
