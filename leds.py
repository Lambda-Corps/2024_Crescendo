import wpilib
from commands2 import Subsystem

#  Implementation of the LEDs on the Robot:
#  LEDs on left side of robot [0-30]    31 LEDs
#  LEDs on right side of robot [31-69]  38 LEDs
#  LEDs on Note Launcher [70-96]        26 LEDs  (Two LED strings receiving same data)


kRightSideBufferStartPoint = 0
kRightSideLEDCount = 31

kLeftSideBufferStartPoint = 31
kLeftSideLEDCount = 38

kLauncherStartPoint = 69
kLauncherLEDCount = 26

kLEDBuffer = kRightSideLEDCount + kLeftSideLEDCount + kLauncherLEDCount


class LEDSubsystem(Subsystem):
    def __init__(self) -> None:
        # PWM Port 9
        self.led = wpilib.AddressableLED(9)

        # LED Data
        self.ledData = [wpilib.AddressableLED.LEDData() for _ in range(kLEDBuffer)]

        # Store what the last hue of the first pixel is
        self.rainbowFirstPixelHue = 0

        # Length is expensive to set, so only set it once, then just update data
        self.led.setLength(kLEDBuffer)

        # Set the data
        self.led.setData(self.ledData)
        self.led.start()

        self._RunwayLightcounter = 0
        self._counter = 0
        self._set_counter = 0
        self._number_of_sets = 5  # seconds to perform color toggle

    def periodic(self) -> None:
        # Fill the buffer with a rainbow
        # self.rainbow()

        # Toggle color every other second for 5 seconds then turn on runway lights
        if self._set_counter < (self._number_of_sets * 50):
            if self._counter == 0:
                self.setRightSideRGB(255, 0, 0)  #  Red - Using RGB
                self.setLeftSideRGB(0, 0, 255)  # Blue - Using RGB
                self.setLauncherRGB(255, 255, 0)  # Yellow - Using RGB
            if self._counter == 50:
                self.setRightSideRGB(0, 0, 255)  #  Blue - Using RGB
                self.setLeftSideRGB(255, 0, 0)  # Red - Using RGB
                self.setLauncherRGB(0, 255, 0)  # Green - Using RGB
            if self._counter == 100:
                self._counter = -1
            self._counter = self._counter + 1

        # if self._set_counter < (self._number_of_sets * 50):
        #     self._set_counter = self._set_counter + 1
        # else:
        #     self.setRightSideRGB(255, 255, 0)  #  Yellow - Using RGB
        #     self.setLeftSideRGB(255, 255, 0)  # Yellow - Using RGB
        #     self.setLauncherRunwayLights()  # Turn on runway lights

        # Set the LEDs
        self.led.setData(self.ledData)

    def setRightSideRGB(self, red: int, green: int, blue: int) -> None:
        for i in range(
            kRightSideBufferStartPoint, kRightSideBufferStartPoint + kRightSideLEDCount
        ):
            self.ledData[i].setRGB(red, green, blue)  #  Using RGB

    def setLeftSideRGB(self, red: int, green: int, blue: int) -> None:
        for i in range(
            kLeftSideBufferStartPoint, kLeftSideBufferStartPoint + kLeftSideLEDCount
        ):
            self.ledData[i].setRGB(red, green, blue)  #  Using RGB

    def setLauncherRGB(self, red: int, green: int, blue: int) -> None:
        for i in range(kLauncherStartPoint, kLauncherStartPoint + kLauncherLEDCount):
            self.ledData[i].setRGB(red, green, blue)  #  Using RGB

    def setLauncherRunwayLights(self) -> None:

        self.ledData[kLauncherStartPoint + self._RunwayLightcounter].setRGB(
            255, 255, 255
        )  # Turn on new light

        # Turn off previous light
        if self._RunwayLightcounter > 0:  # special case - first light
            self.ledData[kLauncherStartPoint + self._RunwayLightcounter - 1].setRGB(
                0, 0, 0
            )  # Turn off previous light
        if self._RunwayLightcounter == 0:  #  special case - last light
            self.ledData[kLauncherStartPoint + kLauncherLEDCount - 1].setRGB(
                0, 0, 0
            )  # Turn off previous light

        if self._RunwayLightcounter < kLauncherLEDCount - 1:
            self._RunwayLightcounter = self._RunwayLightcounter + 1
        else:
            self._RunwayLightcounter = 0

    def rainbow(self) -> None:
        # For every pixel
        for i in range(kLEDBuffer):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = (self.rainbowFirstPixelHue + (i * 180 / kLEDBuffer)) % 180

            # Set the value
            self.ledData[i].setHSV(int(hue), 255, 128)

        # Increase by to make the rainbow "move"
        self.rainbowFirstPixelHue += 3

        # Check bounds
        self.rainbowFirstPixelHue %= 180
