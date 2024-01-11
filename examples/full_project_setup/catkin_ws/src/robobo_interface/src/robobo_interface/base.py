import functools
from abc import ABC, abstractmethod

import numpy

from robobo_interface.datatypes import (
    Emotion,
    LedColor,
    LedId,
    Acceleration,
    Orientation,
    WheelPosition,
    SoundEmotion,
)
from robobo_interface.utils import LockedSet

from typing import List, Optional, Callable
from numpy.typing import NDArray


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

    _used_pids: LockedSet[int]

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
    ) -> int:
        """Move the robot wheels for `millis` time

        Arguments
        left_speed: speed of the left wheel. Range: -100-0-100. 0 is no movement.
        right_speed: speed of the right wheel. Range: -100-0-100. 0 is no movement.
        millis: how many millisecond to move the robot
        blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic.
            Use a value that is within a 16-bit integer limit
            If None is passed, a random available blockid is chosen.

        returns:
            the blockid
        """
        ...

    def move_blocking(self, left_speed: int, right_speed: int, millis: int) -> None:
        """Move the robot wheels for `millis` time

        Arguments
        left_speed: speed of the left wheel. Range: -100-0-100. 0 is no movement, negative backward.
        right_speed: speed of the right wheel. Range: -100-0-100. 0 is no movement, negative backward.
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
        [BackL, BackR, FrontL, FrontR, FrontC, FrontRR, BackC, FrontLL]
        """
        ...

    @abstractmethod
    def get_image_front(self) -> NDArray[numpy.uint8]:
        """Get the image from the front camera as a numpy array in cv2 format.

        You can, for example, write this image to file with:
        https://docs.opencv.org/3.4/d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce
        """
        ...

    @abstractmethod
    def set_phone_pan(
        self, pan_position: int, pan_speed: int, blockid: Optional[int] = None
    ) -> int:
        """Command the robot to move the smartphone holder in the horizontal (pan) axis.
        This function is asyncronous.

        Arguments
        pan_position: Angle to position the pan at. Range: 11-343.
        pan_speed: Movement speed for the pan mechanism. Range: 0-100.
        blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic.
            Use a value that is within a 16-bit integer limit
            If None is passed, a random available blockid is chosen.

        returns:
            the blockid
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
    def read_phone_pan(self) -> int:
        """Get the current pan of the phone. Range: 0-100"""
        ...

    @abstractmethod
    def set_phone_tilt(
        self, tilt_position: int, tilt_speed: int, blockid: Optional[int] = None
    ) -> int:
        """Command the robot to move the smartphone holder in the vertical (tilt) axis.
        This function is asyncronous.

        Arguments
        tilt_position: Angle to position the tilt at. Range: 26-109.
        tilt_speed: Movement speed for the tilt mechanism. Range: 0-100.
        blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic.
            Use a value that is within a 16-bit integer limit
            If None is passed, a random available blockid is chosen.

        returns:
            the blockid
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

    @abstractmethod
    def sleep(self, seconds: float) -> None:
        """Block for a an amount of seconds.
        How to do this depends on the kind of robot, and so is to be found here.
        """
        ...

    def perform_blocking(self, f: Callable[[], int]) -> None:
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
        blockid = f()
        while self.is_blocked(blockid):
            self.sleep(0.002)

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

    def _first_unblocked(self) -> int:
        """Get the first available blockid"""
        return min(set(range(1, 768)) - self._used_pids)
