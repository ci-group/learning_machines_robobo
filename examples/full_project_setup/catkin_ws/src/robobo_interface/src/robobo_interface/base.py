from enum import Enum
from dataclasses import dataclass
from abc import ABC, abstractmethod

import numpy

from typing import List, Optional
from numpy.typing import NDArray


class Emotion(Enum):
    """The emotions the hardware robobo can display
    https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotemotion
    """

    HAPPY = "happy"
    LAUCHING = "laughing"
    SAD = "sad"
    ANGRY = "angry"
    SURPRISED = "surprised"
    NORMAL = "normal"


class LedId(Enum):
    """The valid ids of a Led
    https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotset_led
    """

    FRONTCENTER = "Front-C"
    FRONTRIGHT = "Front-R"
    FRONTRIGHTRIGHT = "Front-RR"
    FRONTLEFT = "Front-L"
    FRONTLEFTLEFT = "Front-LL"
    BACKRIGHT = "Back-R"
    BACKLEFT = "Back-L"


class LedColor(Enum):
    """The valid colour values of a Led
    https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotset_led
    """

    WHITE = "white"
    RED = "red"
    BLUE = "blue"
    CYAN = "cyan"
    MAGENTA = "magenta"
    YELLOW = "yellow"
    GREEN = "green"
    ORANGE = "orange"
    OFF = "off"


@dataclass
class Acceleration:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Orientation:
    """Orientation in Quaternion form"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0


@dataclass
class WheelPosition:
    wheel_pos_r: float = 0.0
    wheel_pos_l: float = 0.0
    wheel_speed_r: float = 0.0
    wheel_speed_l: float = 0.0


class IRobobo(ABC):
    @abstractmethod
    def __init__(self, *args, **kwargs) -> None:
        ...

    @abstractmethod
    def set_emotion(self, emotion: Emotion) -> None:
        """Show the emotion of the robot on the screen

        Arguments

        emotion: Emotion - What emotion to show.
        """
        ...

    @abstractmethod
    def move(
        self, left_speed: int, right_speed: int, millis: int, blockid: int = 0
    ) -> None:
        """Move the robot wheels for `millis` time

        Arguments

        left_speed: speed of the left wheel. Range: 0-100
        right_speed: speed of the right wheel. Range: 0-100
        millis: how many millisecond to move the robot
        """
        ...

    @abstractmethod
    def reset_wheels(self) -> None:
        """Allows to reset the wheel encoder positions to 0.
        After calling this topic both encoders (topic /robot/wheels) will start again
        in position 0.
        """
        ...

    @abstractmethod
    def talk(self, message: str) -> None:
        """Let the robot speak.

        Arguments

        message: str - what to say
        """
        ...

    @abstractmethod
    def set_led(self, selector: LedId, color: LedColor) -> None:
        """Set the led of the robot

        Arguments:
        selector: LedId
        color: LedColor
        """
        ...

    # The returned options here might sometimes be False instead of None.
    # So, check with `if X` not with `if X is not none`
    @abstractmethod
    def read_irs(self) -> List[Optional[float]]:
        """
        Returns sensor readings: [backR, backC, backL, frontRR, frontR, frontC, frontL, frontLL]
        """
        ...

    @abstractmethod
    def get_image_front(self) -> NDArray[numpy.uint8]:
        """Get the image from the front camera as a numpy array"""
        ...

    @abstractmethod
    def set_phone_pan(
        self, pan_position: int, pan_speed: int, pan_blockid: int = 1
    ) -> None:
        """
        Command the robot to move the smartphone holder in the horizontal (pan) axis.
        This function is asyncronous.

        Arguments

        pan_position: Angle to position the pan at. Range: 11-343.
        pan_speed: Movement speed for the pan mechanism. Range: 0-100.
        pan_blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic.
            Must be greater that 0 to move.
        """
        ...

    @abstractmethod
    def read_phone_pan(self) -> int:
        """Get the current pan of the phone. Range: 0-100"""
        ...

    @abstractmethod
    def set_phone_tilt(
        self, tilt_position: int, tilt_speed: int, tilt_blockid: int = 1
    ) -> None:
        """
        Command the robot to move the smartphone holder in the vertical (tilt) axis.
        This function is asyncronous.

        Arguments

        tilt_position: Angle to position the tilt at. Range: 26-109.
        tilt_speed: Movement speed for the tilt mechanism. Range: 0-100.
        tilt_blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic.
            Must be greater that 0 to move.
        """
        ...

    @abstractmethod
    def read_phone_tilt(self) -> int:
        """Get the current tilt of the phone. Range: 26-109"""
        ...

    @abstractmethod
    def read_accel(self) -> Acceleration:
        """Get the acceleration of the robot"""
        ...

    @abstractmethod
    def read_orientation(self) -> Orientation:
        """Get the orientation of the robot"""
        ...

    @abstractmethod
    def read_wheels(self) -> WheelPosition:
        """Get the wheel orientation and speed of the robot"""
        ...

    @abstractmethod
    def sleep(seconds: int) -> None:
        """Block for a an amount of seconds.
        How to do this depends on the kind of robot, and so is to be found here.
        """
        ...
