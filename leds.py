from enum import Enum
from wpilib import AddressableLED, Timer, DriverStation
from commands2 import Subsystem, Command, InstantCommand

from typing import List, Optional, Dict

#  Implementation of the LEDs on the Robot:
#  LEDs on left side of robot [0-30]    31 LEDs
#  LEDs on right side of robot [31-69]  38 LEDs
#  LEDs on Note Launcher [70-96]        26 LEDs  (Two LED strings receiving same data)


kRightSideStart = 0
kRightSideCount = 31

kLeftSideStart = 31
kLeftSideCount = 38

kLauncherStart = 69
kLauncherCount = 26

kLEDTotalCount = kRightSideCount + kLeftSideCount + kLauncherCount

# HSV will be indexed as H=0, S=1, V=2
kGreenRGB: List[int] = [0, 255, 0]
kRedRGB: List[int] = [255, 0, 0]
kBlueRGB: List[int] = [0, 0, 255]
kOrangeRGB: List[int] = [255, 175, 0]
kWhiteRGB: List[int] = [255, 255, 255]

hsv_dict: List[int] = {}
hsv_dict["green"] = kGreenRGB
hsv_dict["red"] = kRedRGB
hsv_dict["blue"] = kBlueRGB
hsv_dict["orange"] = kOrangeRGB
hsv_dict["white"] = kWhiteRGB


class LEDState(Enum):
    DEFAULT = 0
    ALLIANCE_SET = 1
    TRACK_APRIL_TAG = 2
    TRACK_NOTE = 3
    HAS_NOTE = 4
    HAS_CORRECT_TAG = 5
    SHOOTING = 6


class LEDSubsystem(Subsystem):
    def __init__(self) -> None:
        # PWM Port 9
        self.leds = AddressableLED(9)
        # Length is expensive to set, so only set it once, then just update data
        self.leds.setLength(kLEDTotalCount)

        self._RunwayLightcounter = 0
        self._counter = 0
        self._set_counter = 0
        self._number_of_sets = 5  # seconds to perform color toggle
        # Store what the last hue of the first pixel is
        self.rainbowFirstPixelHue = 0

        # Dictionary to hold all the LED buffer possibilities
        self.__chase_buffer_dict = {}
        self.__flash_buffer = [AddressableLED.LEDData() for _ in range(kLEDTotalCount)]
        self.__rainbow_buffer = [
            AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        ]
        # self.rainbow(self.__rainbow_buffer)

        # LED Data
        self.__build_led_data_buffers()

        # Set the data
        self.leds.setData(self.__flash_buffer)
        self.leds.start()

        self._timer = Timer()
        self._timer.start()

        # Variables to store timing data on led animation
        self._last_animate_time = 0
        self._last_flash_time = 0
        self._last_flash_off = True

        # Set the State to default
        self._curr_state = self._last_state = LEDState.ALLIANCE_SET
        self._alliance: Optional[DriverStation.Alliance] = None

    def __build_led_data_buffers(self) -> None:
        """
        Build a set of buffers that will be used for the LEDs.  Using separate buffers for each
        color, animation, etc will minimize the computation needed to switch between schemes
        """
        self.__flash_buffer = [AddressableLED.LEDData() for _ in range(kLEDTotalCount)]

        self.__chase_buffer_dict["blue"] = [
            AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        ]
        self.__initialize_buffer_with_color(
            self.__chase_buffer_dict["blue"], kBlueRGB, 10
        )

        self.__chase_buffer_dict["red"] = [
            AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        ]
        self.__initialize_buffer_with_color(
            self.__chase_buffer_dict["red"], kRedRGB, 10
        )

        # self.__flash_buffer["orange"] = [
        #     AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        # ]
        # self.__initialize_buffer_with_color(
        #     self.__flash_buffer["orange"], kOrangeRGB, kLEDTotalCount, split=False
        # )

        # self.__flash_buffer["white"] = [
        #     AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        # ]
        # self.__initialize_buffer_with_color(
        #     self.__flash_buffer["white"], kWhiteRGB, kLEDTotalCount, split=False
        # )

        # self.__flash_buffer["green"] = [
        #     AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        # ]
        # self.__initialize_buffer_with_color(
        #     self.__flash_buffer["green"], kGreenRGB, kLEDTotalCount, split=False
        # )

    def __initialize_buffer_with_color(
        self,
        buffer: List[AddressableLED.LEDData],
        color: List[int],
        count: int,
        split=True,
    ) -> None:
        if split:
            # If we're split, treat the right side, left side, and launcher
            # ramp as if they were three different LED strips
            for i in range(kLeftSideStart, kLeftSideStart + count):
                buffer[i].setRGB(color[0], color[1], color[2])

            for i in range(kRightSideStart, kRightSideStart + count):
                buffer[i].setRGB(color[0], color[1], color[2])

            for i in range(kLauncherStart, kLauncherStart + count):
                buffer[i].setRGB(color[0], color[1], color[2])
        else:
            # If they aren't split, treat all the LEDs as one continuous
            # chain of LEDs in series.
            for i in range(0, count):
                buffer[i].setRGB(color[0], color[1], color[2])

    def __animate_chase_buffers(self, delay: float) -> None:
        timestamp = self._timer.get()
        if timestamp - delay > self._last_animate_time:
            # Move the LEDs around all chase buffers
            for buffer in self.__chase_buffer_dict.values():
                # LEDS are in an array like this:
                # [ a b c d e ]
                #   0 1 2 3 4  <--- Their index value
                # 0 is the first led
                # Take the first one in the array and save it
                temp = buffer[0]

                # Shift all the LEDs to the left by 1 spot
                for i in range(1, len(buffer)):
                    buffer[i - 1] = buffer[i]

                # Take the first one we stored, and put it on the back of the list
                buffer[len(buffer) - 1] = temp
                # Resulting LED buffer now looks like this:
                # [ b c d e a ]

            # Store the time for the next run
            self._last_animate_time = timestamp

    def __set_flash_buffers_color(self, delay: float, color: List[int]) -> None:
        timestamp = self._timer.get()
        if (timestamp - delay) > self._last_flash_time and self._last_flash_off:
            # Turn the colors on
            for led in self.__flash_buffer:
                led.setRGB(color[0], color[1], color[2])

            self._last_flash_time = timestamp
            self._last_flash_off = False
        else:
            # turn lights off
            for led in self.__flash_buffer:
                led.setRGB(0, 0, 0)

            self._last_flash_off = True

    def periodic(self) -> None:
        # Check for our alliance to match color
        self.check_alliance()

        # Move the leds in their buffers in case they get set
        self.__animate_chase_buffers(0.03)
        self.rainbow(self.__rainbow_buffer)

        # Set the LEDs based on the steate
        match self._curr_state:
            case LEDState.TRACK_APRIL_TAG:
                self.__set_flash_buffers_color(1, kWhiteRGB)
                self.leds.setData(self.__flash_buffer)
            case LEDState.TRACK_NOTE:
                self.__set_flash_buffers_color(1, kOrangeRGB)
                self.leds.setData(self.__flash_buffer)
            case LEDState.HAS_NOTE:
                self.__set_flash_buffers_color(0.2, kOrangeRGB)
                self.leds.setData(self.__flash_buffer)
            case LEDState.HAS_CORRECT_TAG:
                self.__set_flash_buffers_color(0.2, kWhiteRGB)
                self.leds.setData(self.__flash_buffer)
            case LEDState.SHOOTING:
                pass
            case LEDState.ALLIANCE_SET:  # Default
                if self._alliance == DriverStation.Alliance.kBlue:
                    self.leds.setData(self.__chase_buffer_dict["blue"])
                elif self._alliance == DriverStation.Alliance.kRed:
                    self.leds.setData(self.__chase_buffer_dict["red"])
                else:
                    self.leds.setData(self.__rainbow_buffer)

    def rainbow(self, buffer: List[AddressableLED.LEDData]) -> None:
        # For every pixel
        for i in range(kLEDTotalCount):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = (self.rainbowFirstPixelHue + (i * 180 / kLEDTotalCount)) % 180

            # Set the value
            buffer[i].setHSV(int(hue), 255, 128)

        # Increase by to make the rainbow "move"
        self.rainbowFirstPixelHue += 3

        # Check bounds
        self.rainbowFirstPixelHue %= 180

    def check_alliance(self) -> None:
        """
        Poll the driverstation for our alliance, set the member variable if we have one.

        The call will return None if we aren't connected to driverstation
        """
        self._alliance = DriverStation.getAlliance()

    def set_state(self, state: LEDState) -> None:
        self._last_state = self._curr_state
        self._curr_state = state

    def unset_state(self) -> None:
        self._curr_state = self._last_state


class FlashLEDCommand(Command):
    def __init__(self, leds: LEDSubsystem, time: float):
        self._led_sub = leds
        self._duration = time

        self._timer = Timer()
        self._timer.start()

    def initialize(self):
        self._timer.restart()
        self._led_sub.set_state(LEDState.HAS_NOTE)

    def isFinished(self) -> bool:
        return self._timer.hasElapsed(self._duration)

    def end(self, interrupted: bool):
        self._led_sub.unset_state()
