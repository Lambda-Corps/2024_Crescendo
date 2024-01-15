#!/usr/bin/env python3
import wpilib
from commands2 import (
    TimedCommandRobot,
    CommandScheduler,
    Command,
    PrintCommand,
    RunCommand,
    cmd,
)
from commands2.button import CommandXboxController
import navx
import drivetrain
from shooter_test import ShooterTest
from shooter_command import ShooterTestCommand

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
        self._gyro = navx.AHRS.create_spi()

        # Setup the operator interface (typically CommandXboxController)
        self._driver_controller = CommandXboxController(0)

        # Instantiate any subystems
        self._shooter = ShooterTest()

        self._driver_controller.a().whileTrue(ShooterTestCommand(self._shooter))
        self._drivetrain = drivetrain.DriveTrain()

        # # Setup the default commands for subsystems
        self._drivetrain.setDefaultCommand(
            # A split-stick arcade command, with forward/backward controlled by the left
            # hand, and turning controlled by the right.
            RunCommand(
                lambda: self._drivetrain.drive_manually(
                    -self._driver_controller.getRawAxis(1),
                    self._driver_controller.getRawAxis(0),
                ),
                self._drivetrain,
            )
        )

        wpilib.SmartDashboard.putData("Shooter", self._shooter)
        wpilib.SmartDashboard.putData("DriveTrain", self._drivetrain)

        # # # Drive forward at half speed for three seconds
        # self._driver_controller.a().onTrue(
        #     cmd.run(
        #         lambda: self._drivetrain.drive_manually(0.2, 0),
        #         self._drivetrain,
        #     ).withTimeout(3)
        # )
        # # # Drive backward at half speed for three seconds
        # self._driver_controller.b().onTrue(
        #     cmd.run(
        #         lambda: self._drivetrain.drive_manually(-0.2, 0),
        #         self._drivetrain,
        #     ).withTimeout(3)
        # )

        self._auto_command = None

    def getAutonomousCommand(self) -> Command:
        return PrintCommand("Default auto selected")

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
