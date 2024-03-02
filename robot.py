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
    WaitCommand,
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
from intake import Intake, IntakeCommand, DefaultIntakeCommand, EjectNote
from shooter import Shooter, SetShooter, ShooterPosition
from robot_commands import ShootCommand, StopIndexAndShooter
from leds import LEDSubsystem
from climber import Climber, MoveClimber
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

        self._leds: LEDSubsystem = LEDSubsystem()

        self._climber: Climber = Climber()
        wpilib.SmartDashboard.putData("Climber", self._climber)

        self.__configure_default_commands()

        self.__configure_button_bindings()

        self.__configure_autonomous_commands()

        self._auto_command = None
        self._current_pose = Pose2d()

    def __configure_button_bindings(self) -> None:
        # Driver controller controls first
        self._driver_controller.a().whileTrue(IntakeCommand(self._intake))

        # Left Trigger Note Aim
        # Right Trigger April Tag
        self._driver_controller.rightBumper().whileTrue(
            RunCommand(
                lambda: self._drivetrain.drive_teleop(
                    self._driver_controller.getLeftY(),
                    -self._driver_controller.getRightX(),
                ),
                self._drivetrain,
            ).withName("FlippedControls")
        )

        ######################## Partner controller controls #########################
        self._partner_controller.a().onTrue(ShootCommand(self._intake, self._shooter))
        self._partner_controller.x().onTrue(IntakeCommand(self._intake))
        self._partner_controller.y().onTrue(
            StopIndexAndShooter(self._shooter, self._intake)
        )

        # Right Trigger Climber Up
        self._partner_controller.rightTrigger().whileTrue(
            MoveClimber(self._climber, 1).withName("ClimberUp")
        )
        # Left Trigger Climber Down
        self._partner_controller.leftTrigger().whileTrue(
            MoveClimber(self._climber, -1).withName("ClimberDown")
        )
        # Climber up for 10 seconds
        self._partner_controller.back(
            MoveClimber(self._climber, 1, 10).withName("ClimberUp10")
        )
        # Climber down for 10 seconds
        self._partner_controller.start(
            MoveClimber(self._climber, -1, 10).withName("ClimberDown10")
        )

        # Eject Note
        self._partner_controller.leftBumper().whileTrue(EjectNote(self._intake))

        # POV for shooting positions
        self._partner_controller.povLeft().onTrue(
            SetShooter(ShooterPosition.SUBWOOFER_2)
        )
        self._partner_controller.povDown().onTrue(SetShooter(ShooterPosition.RING_2))
        self._partner_controller.povRight().onTrue(SetShooter(ShooterPosition.AMP))

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

        self._intake.setDefaultCommand(DefaultIntakeCommand(self._intake))

    def __configure_autonomous_commands(self) -> None:
        # Register the named commands used by the PathPlanner auto builder
        # These commands have to match exactly in the PathPlanner application
        # as we name them here in the registration
        NamedCommands.registerCommand(
            "AutoShoot", ShootCommand(self._intake, self._shooter)
        )
        NamedCommands.registerCommand(
            "AutoIntake_tm2",
            IntakeCommand(self._intake).withTimeout(2).withName("AutoIntake 2"),
        )

        NamedCommands.registerCommand(
            "AutoIntake_tm3",
            IntakeCommand(self._intake).withTimeout(2).withName("AutoIntake 3"),
        )
        NamedCommands.registerCommand(
            "SetShooterRampToLine",
            cmd.run(
                lambda: self._shooter.set_shooter_location(
                    self._shooter.SPEAKER_FROM_RING2
                ),
                self._shooter,
            )
            .until(self._shooter.shooter_at_angle)
            .andThen(lambda: self._shooter.stop_shooter_ramp(), self._shooter),
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
            [0.0625, 0.125, 2.5],  # <-- Q Elements
            # [0.075, 0.15, 3.1],
            # [0.09, 0.19, 3.7],
            # [0.125, 2.5, 5.0],
            # [0.19, 3.75, 7.5],
            # [2.5, 5.0, 10.0],
            # current [-5, 5],  # <-- R elements
            # [-0.5, 0.5],
            [-10, 10],
            # [-11, 11],
            # [-12, 12],
            0.02,
            # ReplanningConfig(
            #     False, False
            # ),
            ReplanningConfig(),  # Default path replanning config. See the API for the options here
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
        self._auto_chooser.setDefaultOption(
            "Sub 2 - Two Ring", PathPlannerAuto("TwoRingSub2")
        )

        self._auto_chooser.addOption(
            "Turn .1",
            RunCommand(
                lambda: self._drivetrain.drive_volts(-0.1, 0.1), self._drivetrain
            )
            .withTimeout(3)
            .withName("Turn .1 3"),
        )
        self._auto_chooser.addOption(
            "Turn .15",
            RunCommand(
                lambda: self._drivetrain.drive_volts(-0.15, 0.15), self._drivetrain
            )
            .withTimeout(3)
            .withName("Turn .15 3"),
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
        # If we're starting on the blue side, offset the Navx angle by 180
        # so 0 degrees points to the right for NWU
        self._drivetrain.set_alliance_offset()
        self._drivetrain.reset_encoders()

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
