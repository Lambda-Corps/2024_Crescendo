# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
from __future__ import annotations

from typing import Callable, Union, Optional, Tuple, overload
from wpimath.controller import (
    PIDController,
    RamseteController,
    SimpleMotorFeedforwardMeters,
)
from wpimath.geometry import Pose2d
from wpimath.kinematics import (
    ChassisSpeeds,
    DifferentialDriveKinematics,
    DifferentialDriveWheelSpeeds,
)
from wpimath.trajectory import Trajectory
from wpimath import units as units
from wpiutil import SendableBuilder
from wpilib import Timer


from commands2 import Command
from commands2 import Subsystem


class RamseteCommand(Command):
    """
    A command that uses a RAMSETE controller (RamseteController) to follow a trajectory
    (Trajectory) with a differential drive.

    The command handles trajectory-following, PID calculations, and feedforwards internally. This
    is intended to be a more-or-less "complete solution" that can be used by teams without a great
    deal of controls expertise.

    Advanced teams seeking more flexibility (for example, those who wish to use the onboard PID
    functionality of a "smart" motor controller) may use the secondary constructor that omits the PID
    and feedforward functionality, returning only the raw wheel speeds from the RAMSETE controller.
    """

    @overload
    def __init__(
        self,
        trajectory: Trajectory,
        pose: Callable[[], Pose2d],
        controller: RamseteController,
        kinematics: DifferentialDriveKinematics,
        requirements: Tuple[Subsystem],
        *,
        outputMPS: Callable[[units.meters_per_second, units.meters_per_second], None],
    ):
        ...

    @overload
    def __init__(
        self,
        trajectory: Trajectory,
        pose: Callable[[], Pose2d],
        controller: RamseteController,
        kinematics: DifferentialDriveKinematics,
        requirements: Tuple[Subsystem],
        *,
        outputVolts: Callable[[units.volts, units.volts], None],
        feedforward: SimpleMotorFeedforwardMeters,
        leftController: PIDController,
        rightController: PIDController,
        wheelSpeeds: Callable[[], DifferentialDriveWheelSpeeds],
    ):
        ...

    def __init__(
        self,
        trajectory: Trajectory,
        pose: Callable[[], Pose2d],
        controller: RamseteController,
        kinematics: DifferentialDriveKinematics,
        requirements: Tuple[Subsystem],
        *,
        outputMPS: Optional[
            Callable[[units.meters_per_second, units.meters_per_second], None]
        ] = None,
        outputVolts: Optional[Callable[[units.volts, units.volts], None]] = None,
        feedforward: Optional[SimpleMotorFeedforwardMeters] = None,
        leftController: Optional[PIDController] = None,
        rightController: Optional[PIDController] = None,
        wheelSpeeds: Optional[Callable[[], DifferentialDriveWheelSpeeds]] = None,
    ):
        """Constructs a new RamseteCommand that, when executed, will follow the provided trajectory. PID
        control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
        units of volts.

        Note: The controller will *not* set the outputVolts to zero upon completion of the path -
        this is left to the user, since it is not appropriate for paths with nonstationary endstates.

        :param trajectory:      The trajectory to follow.
        :param pose:            A function that supplies the robot pose - use one of the odometry classes to
                                provide this.
        :param controller:      The RAMSETE controller used to follow the trajectory.
        :param kinematics:      The kinematics for the robot drivetrain.
        :param requirements:    The subsystems to require.

        REQUIRED KEYWORD PARAMETERS
        Note: The output mode must be specified by the users of RamseteCommands. Users can specify ONE of
        an output function that will supply the consumer with left and right speeds in `units.meters_per_second`
        or in `units.volts`. If neither, or both, are supplied to the constructor a `RuntimeError` will be
        raised.
        :param outputVolts      A function that consumes the computed left and right outputs in `units.volts`
        :param outputMPS        A function that consumes the computed left and right outputs in `units.meters_per_second`


        Constructs a new RamseteCommand that, when executed, will follow the provided trajectory. PID
        control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
        units of volts.

        Note: The controller will *not* set the outputVolts to zero upon completion of the path -
        this is left to the user, since it is not appropriate for paths with nonstationary endstates.

        :param trajectory:      The trajectory to follow.
        :param pose:            A function that supplies the robot pose - use one of the odometry classes to
                                provide this.
        :param controller:      The RAMSETE controller used to follow the trajectory.
        :param kinematics:      The kinematics for the robot drivetrain.
        :param requirements:    The subsystems to require.

        REQUIRED KEYWORD PARAMETERS
        Note: The output mode must be specified by the users of RamseteCommands. Users can specify ONE of
        an output function that will supply the consumer with left and right speeds in `units.meters_per_second`
        or in `units.volts`. If neither, or both, are supplied to the constructor a `RuntimeError` will be
        raised.
        :param outputVolts      A function that consumes the computed left and right outputs in `units.volts`
        :param outputMPS        A function that consumes the computed left and right outputs in `units.meters_per_second`

        OPTIONAL PARAMETERS
        When the following optional parameters are provided, when executed, the RamseteCommand will follow
        provided trajectory. PID control and feedforward are handled internally, and the outputs are scaled
        from -12 to 12 representing units of volts. If any one of the optional parameters are provided, each
        other optional parameter becomes required.
        :param feedforward      The feedforward to use for the drive
        :param leftController:  The PIDController for the left side of the robot drive.
        :param rightController: The PIDController for the right side of the robot drive.
        :param wheelSpeeds:     A function that supplies the speeds of the left and right sides of the robot
                                drive.
        """
        super().__init__()

        self._timer = Timer()

        # Store all the requirements that are required
        self.trajectory = trajectory
        self.pose = pose
        self.follower = controller
        self.kinematics = kinematics
        self.outputMPS = outputMPS
        self.outputVolts = outputVolts
        self.leftController = leftController
        self.rightController = rightController
        self.wheelspeeds = wheelSpeeds
        self.feedforward = feedforward
        self._prevSpeeds = DifferentialDriveWheelSpeeds()
        self._prevTime = -1.0

        # Required requirements check for output
        if outputMPS is None and outputVolts is None:
            raise RuntimeError(
                f"Failed to instantiate RamseteCommand, no output consumer provided. Must provide either output in meters per second or volts."
            )

        if outputMPS is not None and outputVolts is not None:
            raise RuntimeError(
                f"Failed to instantiate RamseteCommand, too many consumers provided. Must provide either output in meters per second or volts but not both."
            )

        # Optional requirements checks. If any one of the optional parameters are not None, then all the optional
        # requirements become required.
        if (
            feedforward is not None
            or leftController is not None
            or rightController is not None
            or wheelSpeeds is not None
        ):
            if (
                feedforward is None
                or leftController is None
                or rightController is None
                or wheelSpeeds is None
            ):
                raise RuntimeError(
                    f"Failed to instantiate RamseteCommand, not all optional arguments were provided.\n \
                                    Feedforward - {feedforward}, LeftController - {leftController}, RightController - {rightController}, WheelSpeeds - {wheelSpeeds} "
                )
            self.usePID = True
        else:
            self.usePID = False

        # All the parameter checks pass, inform scheduler.  Type ignoring is set explictitly for the linter because this
        # implementation declares the tuple explicitly, whereas the general implementations use the unpack operator (*)
        # to pass the requirements to the scheduler.
        self.addRequirements(requirements)  # type: ignore

    def initialize(self):
        self._prevTime = -1
        initial_state = self.trajectory.sample(0)
        self._prevSpeeds = self.kinematics.toWheelSpeeds(
            ChassisSpeeds(
                initial_state.velocity,
                0,
                initial_state.curvature * initial_state.velocity,
            )
        )
        self._timer.restart()
        if self.usePID:
            self.leftController.reset()
            self.rightController.reset()

    def execute(self):
        curTime = self._timer.get()
        dt = curTime - self._prevTime

        if self._prevTime < 0:
            if self.outputVolts is not None:
                self.outputVolts(0.0, 0.0)
            if self.outputMPS is not None:
                self.outputMPS(0.0, 0.0)
            self._prevTime = curTime
            return

        targetWheelSpeeds = self.kinematics.toWheelSpeeds(
            self.follower.calculate(self.pose(), self.trajectory.sample(curTime))
        )

        leftSpeedSetpoint = targetWheelSpeeds.left
        rightSpeedSetpoint = targetWheelSpeeds.right

        if self.usePID:
            leftFeedforward = self.feedforward.calculate(
                leftSpeedSetpoint, (leftSpeedSetpoint - self._prevSpeeds.left) / dt
            )

            rightFeedforward = self.feedforward.calculate(
                rightSpeedSetpoint,
                (rightSpeedSetpoint - self._prevSpeeds.right) / dt,
            )

            leftOutput = leftFeedforward + self.leftController.calculate(
                self.wheelspeeds().left, leftSpeedSetpoint
            )

            rightOutput = rightFeedforward + self.rightController.calculate(
                self.wheelspeeds().right, rightSpeedSetpoint
            )
            self.outputVolts(leftOutput, rightOutput)
        else:
            leftOutput = leftSpeedSetpoint
            rightOutput = rightSpeedSetpoint
            self.outputMPS(leftOutput, rightOutput)

        self._prevSpeeds = targetWheelSpeeds
        self._prevTime = curTime

    def end(self, interrupted: bool):
        self._timer.stop()

        if interrupted:
            if self.outputVolts is not None:
                self.outputVolts(0.0, 0.0)
            if self.outputMPS is not None:
                self.outputMPS(0.0, 0.0)

    def isFinished(self):
        return self._timer.hasElapsed(self.trajectory.totalTime())

    def initSendable(self, builder: SendableBuilder):
        super().initSendable(builder)
        builder.addDoubleProperty(
            "leftVelocity", lambda: self._prevSpeeds.left, lambda *float: None
        )
        builder.addDoubleProperty(
            "rightVelocity", lambda: self._prevSpeeds.right, lambda *float: None
        )
