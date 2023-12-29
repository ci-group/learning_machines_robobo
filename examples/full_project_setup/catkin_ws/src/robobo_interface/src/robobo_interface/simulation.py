import sys
import time

import cv2
import numpy

from robobo_interface.base import IRobobo
from robobo_interface.datatypes import (
    Emotion,
    LedColor,
    LedId,
    Acceleration,
    Orientation,
    WheelPosition,
    SoundEmotion,
)
from coppelia_sim import sim, simConst

from typing import List, Optional, Callable
from numpy.typing import NDArray


class SimulationRobobo(IRobobo):
    """The simulation robot.

    A few functions take blockid. When they do, they are not blocking.
    However, the simulator does not work with ids, and, therefore, this argument is always ignored.
    """

    def __init__(self):
        sim.simxFinish(-1)  # just in case, close all opened connections
        # 0.0.0.0 to connect to the current computer.
        self.clientID = sim.simxStart("0.0.0.0", 19999, True, True, 5000, 5)
        if self.clientID != -1:
            print("Connected to remote API server")
        else:
            print(
                """Failed connecting to remote API server:
                Cannot find one hosted on port 19999"""
            )
            sys.exit(1)

        self.init_handles()

    def set_emotion(self, emotion: Emotion) -> None:
        """Show the emotion of the robot on the screen
        For the hardware, that means printing... for now.
        The one that was in the example from mintforpeople does not work.

        Arguments
        emotion: Emotion - What emotion to show.
        """
        print(f"The robot shows {emotion.value} on its screen")

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
        blockid: ignored, but implied unblocked.
        """
        inputIntegers = [left_speed, right_speed]
        inputFloats = [millis * 1000.0]
        inputStrings = []
        inputBuffer = bytearray()
        sim.simxCallScriptFunction(
            self.clientID,
            "Left_Motor",
            sim.sim_scripttype_childscript,
            "moveWheelsByTime",
            inputIntegers,
            inputFloats,
            inputStrings,
            inputBuffer,
            sim.simx_opmode_blocking,
        )

    def reset_wheels(self) -> None:
        """Allows to reset the wheel encoder positions to 0.
        After calling this topic both encoders (topic /robot/wheels) will start again
        in position 0.
        """
        inputIntegers = []
        inputFloats = []
        inputStrings = []
        inputBuffer = bytearray()
        sim.simxCallScriptFunction(
            self.clientID,
            "Left_Motor",
            sim.sim_scripttype_childscript,
            "resetWheelEncoders",
            inputIntegers,
            inputFloats,
            inputStrings,
            inputBuffer,
            sim.simx_opmode_blocking,
        )

    def talk(self, message: str) -> None:
        """Let the robot speak.
        For the simultion, this is just printing.

        Arguments
        message: str - what to say
        """
        print(f"The robot says: {message}")

    def play_emotion_sound(self, emotion: SoundEmotion) -> None:
        """Let the robot make an emotion sound
        For the simultion, this is just printing.

        Arguments:
        emotion: SoundEmotion - The sound to make.
        """
        print(f"The robot makes sound: {emotion.value}")

    def set_led(self, selector: LedId, color: LedColor) -> None:
        """Set the led of the robot

        Arguments:
        selector: LedId
        color: LedColor
        """
        inputIntegers = []
        inputFloats = []
        inputStrings = [selector.value, color.value]
        inputBuffer = bytearray()
        sim.simxCallScriptFunction(
            self.clientID,
            "Back_L",
            sim.sim_scripttype_childscript,
            "setLEDColor",
            inputIntegers,
            inputFloats,
            inputStrings,
            inputBuffer,
            sim.simx_opmode_blocking,
        )

    def read_irs(self) -> List[Optional[float]]:
        """Returns sensor readings:
        [backR, backC, backL, frontRR, frontR, frontC, frontL, frontLL]
        """
        inputIntegers = []
        inputFloats = []
        inputStrings = []
        inputBuffer = bytearray()
        array = sim.simxCallScriptFunction(
            self.clientID,
            "IR_Back_C",
            sim.sim_scripttype_childscript,
            "readAllIRSensor",
            inputIntegers,
            inputFloats,
            inputStrings,
            inputBuffer,
            sim.simx_opmode_blocking,
        )
        return list(array[1])

    def get_image_front(self) -> NDArray[numpy.uint8]:
        """Get the image from the front camera as a numpy array"""
        resolution, image = sim.simxReadVisionSensor(
            self.clientID, self._frontal_camera, simConst.simx_opmode_buffer
        )
        # reshape image
        image = image[::-1]
        im_cv2 = numpy.array(image, dtype=numpy.uint8)
        im_cv2.resize([resolution[0], resolution[1], 3])
        im_cv2 = cv2.flip(im_cv2, 1)

        self.block()

        return im_cv2

    def set_phone_pan(
        self, pan_position: int, pan_speed: int, blockid: Optional[int] = None
    ) -> None:
        """Command the robot to move the smartphone holder in the horizontal (pan) axis.
        This function is asyncronous.

        Arguments
        pan_position: Angle to position the pan at. Range: 11-343.
        pan_speed: Movement speed for the pan mechanism. Range: 0-100.
        blockid: ignored, but implied unblocked.
        """
        inputIntegers = [pan_position, pan_speed]
        inputFloats = []
        inputStrings = []
        inputBuffer = bytearray()
        sim.simxCallScriptFunction(
            self.clientID,
            "Pan_Motor",
            sim.sim_scripttype_childscript,
            "movePanTo",
            inputIntegers,
            inputFloats,
            inputStrings,
            inputBuffer,
            sim.simx_opmode_blocking,
        )

    def set_pan_exact(self, pan_position: int = 121) -> None:
        """Command the robot to move the smartphone holder in the horizontal (pan) axis.
        This version tests a bunch to make sure it actually ends up where you think it ends up in the hardware implementation
        Still not perfect for the hardware, and quite slow, but this function might be usefull for calibration / resetting.

        Always blocks.

        Arguments:
        pan: int -> value to move to. Range: 11-343. Defaults to panning to center.
        """
        self.set_phone_pan_blocking(pan_position, 100)

    def read_phone_pan(self) -> int:
        """Get the current pan of the phone. Range: 0-100"""
        inputIntegers = []
        inputFloats = []
        inputStrings = []
        inputBuffer = bytearray()
        array = sim.simxCallScriptFunction(
            self.clientID,
            "Pan_Motor",
            sim.sim_scripttype_childscript,
            "readPanPosition",
            inputIntegers,
            inputFloats,
            inputStrings,
            inputBuffer,
            sim.simx_opmode_blocking,
        )
        return int(array[1][0])

    def set_phone_tilt(
        self, tilt_position: int, tilt_speed: int, blockid: Optional[int] = None
    ) -> None:
        """Command the robot to move the smartphone holder in the vertical (tilt) axis.
        This function is asyncronous.

        Arguments
        tilt_position: Angle to position the tilt at. Range: 26-109.
        tilt_speed: Movement speed for the tilt mechanism. Range: 0-100.
        blockid: ignored, but implied unblocked.
        """
        inputIntegers = [tilt_position, tilt_speed]
        inputFloats = []
        inputStrings = []
        inputBuffer = bytearray()
        sim.simxCallScriptFunction(
            self.clientID,
            "Tilt_Motor",
            sim.sim_scripttype_childscript,
            "moveTiltTo",
            inputIntegers,
            inputFloats,
            inputStrings,
            inputBuffer,
            sim.simx_opmode_blocking,
        )

    def read_phone_tilt(self) -> int:
        """Get the current tilt of the phone. Range: 26-109"""
        inputIntegers = []
        inputFloats = []
        inputStrings = []
        inputBuffer = bytearray()
        array = sim.simxCallScriptFunction(
            self.clientID,
            "Tilt_Motor",
            sim.sim_scripttype_childscript,
            "readTiltPosition",
            inputIntegers,
            inputFloats,
            inputStrings,
            inputBuffer,
            sim.simx_opmode_blocking,
        )
        return int(array[1][0])

    def read_accel(self) -> Acceleration:
        """Get the acceleration of the robot"""
        inputIntegers = []
        inputFloats = []
        inputStrings = []
        inputBuffer = bytearray()
        array = sim.simxCallScriptFunction(
            self.clientID,
            "Smartphone_Respondable",
            sim.sim_scripttype_childscript,
            "readAccelerationSensor",
            inputIntegers,
            inputFloats,
            inputStrings,
            inputBuffer,
            sim.simx_opmode_blocking,
        )
        return Acceleration(*array[2])

    def read_orientation(self) -> Orientation:
        """Get the orientation of the robot"""
        inputIntegers = []
        inputFloats = []
        inputStrings = []
        inputBuffer = bytearray()
        array = sim.simxCallScriptFunction(
            self.clientID,
            "Smartphone_Respondable",
            sim.sim_scripttype_childscript,
            "readOrientationSensor",
            inputIntegers,
            inputFloats,
            inputStrings,
            inputBuffer,
            sim.simx_opmode_blocking,
        )
        return Orientation(*array[2])

    def read_wheels(self) -> WheelPosition:
        """Get the wheel orientation and speed of the robot"""
        inputIntegers = []
        inputFloats = []
        inputStrings = []
        inputBuffer = bytearray()
        array = sim.simxCallScriptFunction(
            self.clientID,
            "Left_Motor",
            sim.sim_scripttype_childscript,
            "readWheels",
            inputIntegers,
            inputFloats,
            inputStrings,
            inputBuffer,
            sim.simx_opmode_blocking,
        )
        return WheelPosition(*array[1])

    def sleep(self, seconds: float) -> None:
        """Block for a an amount of seconds.
        How to do this depends on the kind of robot, and so is to be found here.
        """
        time.sleep(seconds)

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
        f(0)
        self.block()

    def is_blocked(self, blockid: int) -> bool:
        """See if the robot is currently "blocked", which is to say, performing an action

        Arguments:
        blockid: the id to check
        """
        res = sim.simxGetIntegerSignal(
            self.clientID, "Bloqueado", sim.simx_opmode_blocking
        )
        return bool(res[1])

    def block(self):
        """Block untill (only return once) all blocking actions are completed"""
        while self.is_blocked(0):
            self.sleep(0.02)

    def init_handles(self) -> None:
        self._right_motor = sim.simxGetObjectHandle(
            self.clientID, "Right_Motor", simConst.simx_opmode_blocking
        )
        self._left_motor = sim.simxGetObjectHandle(
            self.clientID, "Left_Motor", simConst.simx_opmode_blocking
        )
        self._robobo = sim.simxGetObjectHandle(
            self.clientID, "Robobo", simConst.simx_opmode_blocking
        )

        self._ir_back_c = sim.simxGetObjectHandle(
            self.clientID, "Ir_Back_C", simConst.simx_opmode_blocking
        )
        self._ir_front_c = sim.simxGetObjectHandle(
            self.clientID, "Ir_Front_C", simConst.simx_opmode_blocking
        )
        self._ir_front_ll = sim.simxGetObjectHandle(
            self.clientID, "Ir_Front_LL", simConst.simx_opmode_blocking
        )
        self._ir_front_rr = sim.simxGetObjectHandle(
            self.clientID, "Ir_Front_RR", simConst.simx_opmode_blocking
        )
        self._ir_back_l = sim.simxGetObjectHandle(
            self.clientID, "Ir_Back_L", simConst.simx_opmode_blocking
        )
        self._ir_back_l_floor = sim.simxGetObjectHandle(
            self.clientID, "Ir_Back_L_Floor", simConst.simx_opmode_blocking
        )
        self._ir_back_r = sim.simxGetObjectHandle(
            self.clientID, "Ir_Back_R", simConst.simx_opmode_blocking
        )
        self._ir_back_r_floor = sim.simxGetObjectHandle(
            self.clientID, "Ir_Back_R_Floor", simConst.simx_opmode_blocking
        )
        self._ir_front_l = sim.simxGetObjectHandle(
            self.clientID, "Ir_Front_L", simConst.simx_opmode_blocking
        )
        self._ir_front_l_floor = sim.simxGetObjectHandle(
            self.clientID, "Ir_Front_L_Floor", simConst.simx_opmode_blocking
        )
        self._ir_front_r = sim.simxGetObjectHandle(
            self.clientID, "Ir_Front_R", simConst.simx_opmode_blocking
        )
        self._ir_front_r_floor = sim.simxGetObjectHandle(
            self.clientID, "Ir_Front_R_Floor", simConst.simx_opmode_blocking
        )
        self._tilt_motor = sim.simxGetObjectHandle(
            self.clientID, "Tilt_Motor", simConst.simx_opmode_blocking
        )
        self._pan_motor = sim.simxGetObjectHandle(
            self.clientID, "Pan_Motor", simConst.simx_opmode_blocking
        )

        self._frontal_camera = sim.simxGetObjectHandle(
            self.clientID, "Frontal_Camera", simConst.simx_opmode_blocking
        )
