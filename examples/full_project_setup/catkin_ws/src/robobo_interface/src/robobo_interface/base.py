import functools
from enum import Enum
from dataclasses import dataclass
from abc import ABC, abstractmethod

import numpy

from typing import List, Optional, Callable
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


@dataclass
class Orientation:
    """Orientation in Quaternion form"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0


@dataclass
class WheelPosition:
    """Wheel position of the robot."""

    wheel_pos_r: float = 0.0
    wheel_pos_l: float = 0.0
    wheel_speed_r: float = 0.0
    wheel_speed_l: float = 0.0


class IRobobo(ABC):
    """The interface / template method class of the Robobo.
    It is advised to only use methods defined in this class when writing code,
    both for the hardware and for the simulation.

    A bunch of methods deal with "blocking", which is how to do simulatanious tasks.
    This is quite complex, and has differences between behavior of harware and simulation.

    To avoid this, never use a method that has a `blockid` perameter.
    All of these have sister methods that end in `_blocking` instead.
    This will prevent you from doing simulatanious tasks, but decrease complexity.
    """

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
        self,
        left_speed: int,
        right_speed: int,
        millis: int,
        blockid: Optional[int] = None,
    ) -> None:
        """Move the robot wheels for `millis` time

        Arguments
        left_speed: speed of the left wheel. Range: 0-100
        right_speed: speed of the right wheel. Range: 0-100
        millis: how many millisecond to move the robot
        blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic.
            Use a value that is within a 16-bit integer limit
            If None is passed, a random available blockid is chosen.
        """
        ...

    def move_blocking(self, left_speed: int, right_speed: int, millis: int) -> None:
        """Move the robot wheels for `millis` time

        Arguments
        left_speed: speed of the left wheel. Range: 0-100
        right_speed: speed of the right wheel. Range: 0-100
        millis: how many millisecond to move the robot
        """
        self.perform_blocking(
            functools.partial(self.move, left_speed, right_speed, millis)
        )

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
    def play_emotion_sound(self, emotion: SoundEmotion) -> None:
        """Let the robot make an emotion sound

        Arguments:
        emotion: SoundEmotion - The sound to make.
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
        """Returns sensor readings:
        [backR, backC, backL, frontRR, frontR, frontC, frontL, frontLL]
        """
        ...

    @abstractmethod
    def get_image_front(self) -> NDArray[numpy.uint8]:
        """Get the image from the front camera as a numpy array"""
        ...

    @abstractmethod
    def set_phone_pan(
        self, pan_position: int, pan_speed: int, blockid: Optional[int] = None
    ) -> None:
        """Command the robot to move the smartphone holder in the horizontal (pan) axis.
        This function is asyncronous.

        Arguments
        pan_position: Angle to position the pan at. Range: 11-343.
        pan_speed: Movement speed for the pan mechanism. Range: 0-100.
        blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic.
            Use a value that is within a 16-bit integer limit
            If None is passed, a random available blockid is chosen.
        """
        ...

    def set_phone_pan_blocking(self, pan_position: int, pan_speed: int) -> None:
        """Command the robot to move the smartphone holder in the horizontal (pan) axis.
        This function is asyncronous.

        Arguments
        pan_position: Angle to position the pan at. Range: 11-343.
        pan_speed: Movement speed for the pan mechanism. Range: 0-100.
        """
        self.perform_blocking(
            functools.partial(self.set_phone_pan, pan_position, pan_speed)
        )

    @abstractmethod
    def set_pan_exact(self, pan_position: int = 121) -> None:
        """Command the robot to move the smartphone holder in the horizontal (pan) axis.
        This version tests a bunch to make sure it actually ends up where you think it ends up in the hardware implementation
        Still not perfect for the hardware, and quite slow, but this function might be usefull for calibration / resetting.

        Always blocks.

        Arguments:
        pan: int -> value to move to. Range: 11-343. Defaults to panning to center.
        """
        ...

    @abstractmethod
    def read_phone_pan(self) -> int:
        """Get the current pan of the phone. Range: 0-100"""
        ...

    @abstractmethod
    def set_phone_tilt(
        self, tilt_position: int, tilt_speed: int, blockid: Optional[int] = None
    ) -> None:
        """Command the robot to move the smartphone holder in the vertical (tilt) axis.
        This function is asyncronous.

        Arguments
        tilt_position: Angle to position the tilt at. Range: 26-109.
        tilt_speed: Movement speed for the tilt mechanism. Range: 0-100.
        blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic.
            Use a value that is within a 16-bit integer limit
            If None is passed, a random available blockid is chosen.
        """
        ...

    def set_phone_tilt_blocking(self, tilt_position: int, tilt_speed: int) -> None:
        """Command the robot to move the smartphone holder in the vertical (tilt) axis.
        This function is asyncronous.

        Arguments
        tilt_position: Angle to position the tilt at. Range: 26-109.
        tilt_speed: Movement speed for the tilt mechanism. Range: 0-100.
        """
        self.perform_blocking(
            functools.partial(self.set_phone_tilt, tilt_position, tilt_speed)
        )

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

    @staticmethod
    @abstractmethod
    def sleep(seconds: float) -> None:
        """Block for a an amount of seconds.
        How to do this depends on the kind of robot, and so is to be found here.
        """
        ...

    @abstractmethod
    def perform_blocking(self, f: Callable[[int], None]) -> None:
        """Perform a function in a blocking manner.
        Which is to say, only return once the action is completed.
        Usefull for all functions that take a blockid argument.

        To call this with a function, use partually applied versions. Pass all arguments
        except the blockid, which will be provided by this function.
        example:
        `rob.perform_blocking(functools.partial(rob.move, 10, 100, 250))`

        Arguments:
        f: Callable[[int], None]. Some function to call.
        """
        ...

    @abstractmethod
    def is_blocked(self, blockid: int) -> bool:
        """See if the robot is currently "blocked", which is to say, performing an action

        Arguments:
        blockid: the id to check
        """
        ...

    @abstractmethod
    def block(self) -> None:
        """Block untill (only return once) all blocking actions are completed"""
        ...
