import commands2
import wpilib.drive

# Import the subsystem
from drivetrain import DriveTrain

### Command to follow AprilTag in Autonomous mode

class FollowAprilTag(commands2.CommandBase):
    def __init__(self, drivetrain: DriveTrain) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)


    def initialize(self) -> None:
        print ("Started AUTO ")

    def execute(self) -> None:
        
        # Max yaw is about 20 degrees, 
        apriltag_present = self.drivetrain.get_Apriltag_status()
        turn = self.drivetrain.get_Apriltag_yaw() /100

        wpilib.SmartDashboard.putNumber(
            "Autonomous turn", turn
        )


        if apriltag_present:
            self.drivetrain.drive_teleop(turn, 0.1)    # (Turn , forward)  << This is not correct
        else:
            self.drivetrain.drive_teleop(0.0, 0.0)

 
    def end(self, interrupted: bool) -> None:
        self.drivetrain.drive_teleop(0.0, 0.0)    # Stop the robot
        

    def isFinished(self) -> bool:
        # This command should be triggered while a button is held
        # so we don't want it to finish on it's own.  So always return
        # false to keep the command running until the button is released.
        return False

