from enum import Enum
from math import isclose
from dataclasses import dataclass


# The simulation can be quite imprecise, not to speak about the hardware.
TOLERANCE = 0.1


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


class SoundEmotion(Enum):
    """The emotion the hardware robot can make a sound of
    https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Services#robotplaysound
    """

    MOAN = "moan"
    PURR = "purr"
    ANGRY = "angry"
    APPROVE = "approve"
    DISAPPROVE = "disapprove"
    DISCOMFORT = "discomfort"
    DOUBTFUL = "doubtful"
    LAUGH = "laugh"
    LIKES = "likes"
    MUMBLE = "mumble"
    OUCH = "ouch"
    THINKING = "thinking"


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
    """Acceleration of the robot"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, Position):
            return (
                isclose(self.x, __o.x, abs_tol=TOLERANCE)
                and isclose(self.y, __o.y, abs_tol=TOLERANCE)
                and isclose(self.z, __o.z, abs_tol=TOLERANCE)
            )
        return NotImplemented


@dataclass
class Position:
    """A basic 3d vector to represent a position"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, Position):
            return (
                isclose(self.x, __o.x, abs_tol=TOLERANCE)
                and isclose(self.y, __o.y, abs_tol=TOLERANCE)
                and isclose(self.z, __o.z, abs_tol=TOLERANCE)
            )
        return NotImplemented


@dataclass
class Orientation:
    """Orientation in Yaw, Pitch, Roll"""

    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, Orientation):
            return (
                isclose(self.yaw, __o.yaw, abs_tol=TOLERANCE)
                and isclose(self.pitch, __o.pitch, abs_tol=TOLERANCE)
                and isclose(self.roll, __o.roll, abs_tol=TOLERANCE)
            )
        return NotImplemented


@dataclass
class WheelPosition:
    """Wheel position of the robot."""

    wheel_pos_r: float = 0.0
    wheel_pos_l: float = 0.0
    wheel_speed_r: float = 0.0
    wheel_speed_l: float = 0.0

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, WheelPosition):
            return (
                isclose(self.wheel_pos_r, __o.wheel_pos_r, abs_tol=TOLERANCE)
                and isclose(self.wheel_pos_l, __o.wheel_pos_l, abs_tol=TOLERANCE)
                and isclose(self.wheel_speed_r, __o.wheel_speed_r, abs_tol=TOLERANCE)
                and isclose(self.wheel_speed_l, __o.wheel_speed_l, abs_tol=TOLERANCE)
            )
        return NotImplemented
