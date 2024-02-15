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
from drivetrain import DriveTrain
from intake import Intake, IntakeTestCommand
from shooter import Shooter, ShooterTestCommand
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
        self._driver_controller = CommandXboxController(
            constants.CONTROLLER_DRIVER_PORT
        )
        self._partner_controller = CommandXboxController(
            constants.CONTROLLER_PARTNER_PORT
        )

        # Instantiate any subystems
        self._drivetrain: DriveTrain = DriveTrain()
        wpilib.SmartDashboard.putData("Drivetrain", self._drivetrain)

        self._intake: Intake = Intake()
        wpilib.SmartDashboard.putData("Intake", self._intake)

        self._shooter: Shooter = Shooter()
        wpilib.SmartDashboard.putData("Shooter", self._shooter)

        self.__configure_default_commands()

        self.__configure_button_bindings()

        self.__configure_autonomous_commands()

        self._auto_command = None
        self._current_pose = Pose2d()

    def __configure_button_bindings(self) -> None:
        # Driver controller controls first
        self._driver_controller.a().whileTrue(IntakeTestCommand(self._intake))

        self._driver_controller.b().whileTrue(ShooterTestCommand(self._shooter))

        # Partner controller controls
        self._partner_controller.a().onTrue(
            cmd.run(
                # Spin up the shooter until the speed is correct, then run
                # the indexer to move the note into the flywheels for 1 second
                lambda: self._shooter.drive_motors(),
                self._shooter,
            )
            .until(self._shooter.shooter_at_speed)
            .andThen(self._intake.index_note(0.4))
            .withName("ShootNote")
        )
        self._partner_controller.b().onTrue(
            cmd.runOnce(lambda: self._shooter.stop_motors(), self._shooter)
        )
        self._partner_controller.x().onTrue(
            cmd.run(  # Subsystem "do stuff", basically execute
                lambda: self._intake.drive_index(0.4), self._intake
            )
            .until(  # Subsystem "stop", what would be in isFinished()
                self._intake.has_note
            )  # Give it a name so we see what command is running
            .withName("IntakeNote")
        )

        self._partner_controller.y().onTrue(
            # Stop all indexer motors
            cmd.runOnce(lambda: self._intake.stop_indexer(), self._intake).withName(
                "StopIndexer"
            )
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
                        -self._driver_controller.getLeftY(),
                        -self._driver_controller.getRightX(),
                    ),
                    self._drivetrain,
                ).withName("DefaultDrive")
            )

        self._shooter.setDefaultCommand(
            RunCommand(
                lambda: self._shooter.drive_shooter_ramp(
                    -self._partner_controller.getLeftY()
                ),
                self._shooter,
            ).withName("ShooterDefault")
        )

    def __configure_autonomous_commands(self) -> None:
        # Register the named commands used by the PathPlanner auto builder
        # These commands have to match exactly in the PathPlanner application
        # as we name them here in the registration
        NamedCommands.registerCommand(
            "ShootSub", PrintCommand("Shoot Note into Speaker")
        )
        NamedCommands.registerCommand("StartIntake", PrintCommand("StartIntake"))
        NamedCommands.registerCommand(
            "FeedShooter", PrintCommand("Move Note into Shooter")
        )

        # increasing Qelems numbers, tries to drive more conservatively as the effect
        # In the math, what we're doing is weighting the error less heavily, meaning,
        # as the error gets larger don't react as much.  This makes the robot drive
        # conservatively along the path.
        # Decreasing Relems should make the motors drive less aggressively (fewer volts)
        # In the math, this is the same as increasing Q values.  Basically, think of it
        # like a car, if you limit how far you can press the gas pedal, a driver
        # has a better chance of keeping the car under control
        # Down below, in comments, there are a few candidate values that have been used
        # under testing.  Tweak, and test, to find the right ones.
        AutoBuilder.configureLTV(
            self._drivetrain.get_robot_pose,
            self._drivetrain.reset_odometry,
            self._drivetrain.get_wheel_speeds,  # Current ChassisSpeeds supplier
            self._drivetrain.driveSpeeds,  # Method that will drive the robot given ChassisSpeeds
            # [0.0625, 0.125, 2.5],  # <-- Q Elements
            [0.075, 0.15, 3.1],
            # [0.09, 0.19, 3.7],
            # [0.125, 2.5, 5.0],
            # [0.19, 3.75, 7.5],
            # [2.5, 5.0, 10.0],
            # current [-5, 5],  # <-- R elements
            [-0.5, 0.5],
            # [-10, 10],
            # [-11, 11],
            # [-12, 12],
            0.02,
            ReplanningConfig(
                False, False
            ),  # Default path replanning config. See the API for the options here
            self._drivetrain.should_flip_path,  # Flip if we're on the red side
            self._drivetrain,  # Reference to this subsystem to set requirements
        )

        # To configure the Autonomous routines use PathPlanner to define the auto routines
        # Then, take all of the path planner created routines and add them to the auto
        # chooser so the drive team can select the starting auto.
        self._auto_chooser: wpilib.SendableChooser = wpilib.SendableChooser()
        self._auto_chooser.setDefaultOption(
            "Sub 2 - One Ring", PathPlannerAuto("OneRingSub2")
        )
        self._auto_chooser.addOption("Sub 2 - Two Ring", PathPlannerAuto("TwoRingSub2"))
        self._auto_chooser.addOption("Sub 1 - One Ring", PathPlannerAuto("OneRingSub1"))
        self._auto_chooser.addOption("Sub 1 - Two Ring", PathPlannerAuto("TwoRingSub1"))
        self._auto_chooser.addOption(
            "Sub 1 - Two Ring Long", PathPlannerAuto("TwoRingSub1Long")
        )

        wpilib.SmartDashboard.putData("AutoChooser", self._auto_chooser)

    def getAutonomousCommand(self) -> Command:
        return self._auto_chooser.getSelected()

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
