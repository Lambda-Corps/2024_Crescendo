import wpilib
import commands2
import commands2.cmd
import wpimath.controller
import constants

from drivetrain import DriveTrain
from constants import DriveTrain


import constants


class TurnToAngle(commands2.PIDCommand):

    def __init__(self, targetAngleDegrees: float, drive: DriveTrain) -> None:
        
        super().__init__(
            wpimath.controller.PIDController(
                constants.DriveConstants.kTurnP,
                constants.DriveConstants.kTurnI,
                constants.DriveConstants.kTurnD,
            ),
            
            drive.getHeading,
            
            targetAngleDegrees,
            
            lambda output: drive.driveManually(0, output),
            
            [drive],
        )

        
        self.getController().enableContinuousInput(-180, 180)

  
        self.getController().setTolerance(
            constants.DriveConstants.kTurnToleranceDeg,
            constants.DriveConstants.kTurnRateToleranceDegPerS,
        )

    def isFinished(self) -> bool:
        
        return self.getController().atSetpoint()
