from commands2 import Subsystem, Command, cmd
from phoenix5 import TalonSRX, TalonSRXControlMode
from wpilib import SmartDashboard, AnalogInput, RobotBase
from wpilib.simulation import AnalogInputSim

import constants


class Intake(Subsystem):
    """
    Intake subsystem, uses two different roller sets to pick notes up off the floor
    and then shoot them when needed
    """

    DETECTION_VOLTS_LOWER_BOUND = 1.5
    DETECTION_VOLTS_UPPER_BOUND = 4.0
    BACKUP_NOTE_SPEED = -0.2
    INDEX_NOTE_SPEED = 0.8
    INTAKE_NOTE_SPEED = 0.8

    def __init__(self, test_mode=False):
        super().__init__()

        self._intakeroller = TalonSRX(constants.INTAKE_ROLLER)
        self._intakeroller.configFactoryDefault()
        self._indexroller = TalonSRX(constants.INDEX_ROLLER)
        self._indexroller.configFactoryDefault()
        self._indexroller.setInverted(True)

        # TODO -- Need to set current limits here
        self.__test_mode = test_mode
        if self.__test_mode:
            SmartDashboard.putNumber("IntakeSpeed", 0.6)
            SmartDashboard.putNumber("IndexSpeed", 0.5)

        self._detector_0: AnalogInput = AnalogInput(constants.INTAKE_BEAM_BREAK_0)
        self._detector_1: AnalogInput = AnalogInput(constants.INTAKE_BEAM_BREAK_1)

        if RobotBase.isSimulation():
            SmartDashboard.putNumber("SimVolts", 0)
            self._simAnalogInput: AnalogInputSim = AnalogInputSim(1)

        # Flag to denote whether or not we are shooting
        self.__is_shooting = False

    def drive_index_backward(self):
        index_speed = self.BACKUP_NOTE_SPEED
        intake_speed = self.BACKUP_NOTE_SPEED
        if self.__test_mode:
            index_speed = -SmartDashboard.getNumber("IndexSpeed", 0) / 2
            intake_speed = -SmartDashboard.getNumber("IntakeSpeed", 0) / 2

        self._indexroller.set(TalonSRXControlMode.PercentOutput, index_speed)
        self._intakeroller.set(TalonSRXControlMode.PercentOutput, intake_speed)

    def drive_index(self, shooting=False, doubleshooting=False):
        index_speed = self.INDEX_NOTE_SPEED
        intake_speed = self.INTAKE_NOTE_SPEED
        if self.__test_mode:
            index_speed = SmartDashboard.getNumber("IndexSpeed", 0)
            intake_speed = SmartDashboard.getNumber("IntakeSpeed", 0)
        if shooting:
            index_speed = 1.0
            intake_speed = self.INTAKE_NOTE_SPEED if doubleshooting else 0

        self._indexroller.set(TalonSRXControlMode.PercentOutput, index_speed)
        self._intakeroller.set(TalonSRXControlMode.PercentOutput, intake_speed)

    def stop_indexer(self) -> None:
        self._indexroller.set(TalonSRXControlMode.PercentOutput, 0)
        self._intakeroller.set(TalonSRXControlMode.PercentOutput, 0)

    def has_note(self) -> bool:
        # volts_0 = self._detector_0.getAverageVoltage()
        volts_1 = self._detector_1.getAverageVoltage()

        # return (
        #     (
        #         volts_0 > self.DETECTION_VOLTS_LOWER_BOUND
        #         and volts_1 < self.DETECTION_VOLTS_UPPER_BOUND
        #     )
        #     or (volts_1 > self.DETECTION_VOLTS_LOWER_BOUND)
        #     and (volts_1 < self.DETECTION_VOLTS_UPPER_BOUND)
        # )
        return (volts_1 > self.DETECTION_VOLTS_LOWER_BOUND) and (
            volts_1 < self.DETECTION_VOLTS_UPPER_BOUND
        )

    def index_note(self, speed: float) -> Command:
        return cmd.run(lambda: self.drive_index()).withTimeout(1).withName("IndexNote")

    def simulationPeriodic(self) -> None:
        self._simAnalogInput.setVoltage(SmartDashboard.getNumber("SimVolts", 0))

    def periodic(self) -> None:
        # SmartDashboard.putNumber("RangeVoltage_0", self._detector_0.getAverageVoltage())
        SmartDashboard.putNumber("RangeVoltage_1", self._detector_1.getAverageVoltage())

        # We need to keep the note from touching the shooter wheels on intake
        # if we are detecting the note, drive the wheels backward
        if self.__is_shooting is False:
            if self.has_note():
                self._indexroller.set(
                    TalonSRXControlMode.PercentOutput, self.BACKUP_NOTE_SPEED
                )
            else:
                self._indexroller.set(TalonSRXControlMode.PercentOutput, 0)

    def set_shooting_flag(self, is_shooting: bool) -> None:
        self.__is_shooting = is_shooting


class IntakeCommand(Command):
    """
    Command to run motors of the shooter with a button press
    """

    def __init__(self, intake: Intake):
        super().__init__()
        self._speed = 0
        self._sub = intake

        self.addRequirements(self._sub)

    def initialize(self):
        # self._speed = SmartDashboard.getNumber("IntakeSpeed", 0.3)
        pass

    def execute(self):
        self._sub.drive_index()

    def isFinished(self) -> bool:
        return self._sub.has_note()

    def end(self, interrupted: bool):
        self._sub.stop_indexer()


class DefaultIntakeCommand(Command):
    """
    Default command for the intake.  The only purpose of this command is to keep the motors
    at 0, unless we have a note too high in the shooter bed that needs to be moved down.

    Using this default command instead of periodic, so that this logic doesn't happen when
    the shoot note commands are running.
    """

    def __init__(self, sub: Intake):
        super().__init__()

        self._intake = sub

        self.addRequirements(self._intake)

    def execute(self):
        SmartDashboard.putNumber(
            "RangeVoltage_0", self._intake._detector_0.getAverageVoltage()
        )
        SmartDashboard.putNumber(
            "RangeVoltage_1", self._intake._detector_1.getAverageVoltage()
        )

        # We need to keep the note from touching the shooter wheels on intake
        # if we are detecting the note, drive the wheels backward
        if self._intake.has_note():
            self._intake._indexroller.set(
                TalonSRXControlMode.PercentOutput, self._intake.BACKUP_NOTE_SPEED
            )
        else:
            self._intake._indexroller.set(TalonSRXControlMode.PercentOutput, 0)

    def runsWhenDisabled(self) -> bool:
        return True


class EjectNote(Command):
    def __init__(self, intake: Intake):
        super().__init__()

        self._intake = intake

        self.addRequirements(self._intake)

    def execute(self):
        self._intake.drive_index_backward()

    # IsFinished returns False, should run while held only
    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._intake.stop_indexer()
