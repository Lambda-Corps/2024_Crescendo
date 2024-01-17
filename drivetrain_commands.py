from commands2 import Command

from drivetrain import DriveTrain


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
