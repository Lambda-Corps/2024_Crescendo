from enum import Enum
from wpilib import AddressableLED, Timer, DriverStation
from commands2 import Subsystem

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
        self.__buffer_dict = {}

        # LED Data
        self.__build_led_data_buffers()

        # Set the data
        self.leds.setData(self.__buffer_dict["blank"])
        self.leds.start()

        self._timer = Timer()
        self._timer.start()

        self._last_animate_time = 0

        # Set the State to default
        self._curr_state = self._last_state = LEDState.DEFAULT
        self._alliance: Optional[DriverStation.Alliance] = None

    def __build_led_data_buffers(self) -> None:
        """
        Build a set of buffers that will be used for the LEDs.  Using separate buffers for each
        color, animation, etc will minimize the computation needed to switch between schemes
        """
        self.__buffer_dict["blank"] = [
            AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        ]

        self.__buffer_dict["blue"] = [
            AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        ]
        self.__initialize_buffer_with_color(self.__buffer_dict["blue"], kBlueRGB, 5)

        self.__buffer_dict["red"] = [
            AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        ]
        self.__initialize_buffer_with_color(
            self.__buffer_dict["red"], kRedRGB, 10, split=False
        )

        self.__buffer_dict["rainbow"] = [
            AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        ]
        self.rainbow(self.__buffer_dict["rainbow"])

        self.__buffer_dict["orange"] = [
            AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        ]
        self.__initialize_buffer_with_color(self.__buffer_dict["orange"], kOrangeRGB, 5)

        self.__buffer_dict["white"] = [
            AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        ]
        self.__initialize_buffer_with_color(self.__buffer_dict["white"], kWhiteRGB, 5)

        self.__buffer_dict["green"] = [
            AddressableLED.LEDData() for _ in range(kLEDTotalCount)
        ]
        self.__initialize_buffer_with_color(self.__buffer_dict["green"], kGreenRGB, 5)

    def __initialize_buffer_with_color(
        self,
        buffer: List[AddressableLED.LEDData],
        color: List[int],
        count: int,
        split=True,
    ) -> None:
        if split:
            for i in range(kLeftSideStart, kLeftSideStart + count):
                buffer[i].setRGB(color[0], color[1], color[2])

            for i in range(kRightSideStart, kRightSideStart + count):
                buffer[i].setRGB(color[0], color[1], color[2])

            for i in range(kLauncherStart, kLauncherStart + count):
                buffer[i].setRGB(color[0], color[1], color[2])

        else:
            for i in range(0, count):
                buffer[i].setRGB(color[0], color[1], color[2])

    def __animate_buffers(self, delay: float) -> None:
        timestamp = self._timer.get()
        if timestamp - delay > self._last_animate_time:
            # buffer = buffer[1:] + buffer[:1]
            for buffer in self.__buffer_dict.values():
                temp = buffer[0]
                for i in range(1, len(buffer)):
                    buffer[i - 1] = buffer[i]

                buffer[len(buffer) - 1] = temp

            self._last_animate_time = timestamp

        # return buffer

    def periodic(self) -> None:
        # Check for our alliance to match color

        self.check_alliance()

        self.__animate_buffers(0.03)
        # Animate the animated buffers (red, blue, rainbow)
        # self.__buffer_dict["blue"] = self.__animate_buffer(
        #     self.__buffer_dict["blue"], 0.05
        # )
        # self.__buffer_dict["red"] = self.__animate_buffer(
        #     self.__buffer_dict["red"], 0.05
        # )

        self.rainbow(self.__buffer_dict["rainbow"])

        # Set the LEDs based on the steate
        match self._curr_state:
            case LEDState.ALLIANCE_SET:
                pass
            case LEDState.TRACK_APRIL_TAG:
                pass
            case LEDState.TRACK_NOTE:
                pass
            case LEDState.HAS_NOTE:
                pass
            case LEDState.HAS_CORRECT_TAG:
                pass
            case LEDState.SHOOTING:
                pass
            case _:  # Default
                if self._alliance == DriverStation.Alliance.kBlue:
                    self.leds.setData(self.__buffer_dict["blue"])
                elif self._alliance == DriverStation.Alliance.kRed:
                    self.leds.setData(self.__buffer_dict["red"])
                else:
                    self.leds.setData(self.__buffer_dict["rainbow"])

    def setRightSideRGB(self, red: int, green: int, blue: int) -> None:
        for i in range(kRightSideStart, kRightSideStart + kRightSideCount):
            self.ledData[i].setRGB(red, green, blue)  #  Using RGB

    def setLeftSideRGB(self, red: int, green: int, blue: int) -> None:
        for i in range(kLeftSideStart, kLeftSideStart + kLeftSideCount):
            self.ledData[i].setRGB(red, green, blue)  #  Using RGB

    def setLauncherRGB(self, red: int, green: int, blue: int) -> None:
        for i in range(kLauncherStart, kLauncherStart + kLauncherCount):
            self.ledData[i].setRGB(red, green, blue)  #  Using RGB

    def setLauncherRunwayLights(self) -> None:

        self.ledData[kLauncherStart + self._RunwayLightcounter].setRGB(
            255, 255, 255
        )  # Turn on new light

        # Turn off previous light
        if self._RunwayLightcounter > 0:  # special case - first light
            self.ledData[kLauncherStart + self._RunwayLightcounter - 1].setRGB(
                0, 0, 0
            )  # Turn off previous light
        if self._RunwayLightcounter == 0:  #  special case - last light
            self.ledData[kLauncherStart + kLauncherCount - 1].setRGB(
                0, 0, 0
            )  # Turn off previous light

        if self._RunwayLightcounter < kLauncherCount - 1:
            self._RunwayLightcounter = self._RunwayLightcounter + 1
        else:
            self._RunwayLightcounter = 0

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
