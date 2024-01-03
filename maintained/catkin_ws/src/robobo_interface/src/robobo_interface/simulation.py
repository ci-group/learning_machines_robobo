import os
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
from coppelia_sim import sim, simConst, ping

from typing import List, Optional, Callable
from numpy.typing import NDArray


class SimulationRobobo(IRobobo):
    """The simulation robot.

    The simulation needs to be running for the init to be callable
    (as, otherwise, the api server is not running)

    A few functions take blockid. When they do, they are not blocking.
    However, the simulator does not work with ids, and, therefore, this argument is always ignored.

    Since the behavior of non-blocking functions is different from Simulation to Hardware,
    it is recommended to only use the `_blocking` functions of the robot,
    which are inherited from the IRobobo in the format of a template method.
    """

    def __init__(self):
        sim.simxFinish(-1)  # just in case, close all opened connections
        # 0.0.0.0 to connect to the current computer on Linux, with `--net=host`
        # This doesn't work on Windows or MacOS. There, the variable needs to be specified.
        ip_adress = os.getenv("COPPELIA_SIM_IP", "0.0.0.0")
        self.clientID = sim.simxStart(ip_adress, 19999, True, True, 5000, 5)
        if self.clientID == -1:
            print(
                """Failed connecting to remote API server:
                Cannot find one hosted on port 19999
                Is the simulation running / playing?

                If not on Linux with --net=host:
                Did you specify the IP adress of your computer in scripts/setup.bash?
                """
            )
            sys.exit(1)

        ping.ping(self.clientID)
        print("Connected to remote API server")

    def __del__(self):
        sim.simxFinish(self.clientID)

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
        sim.simxCallScriptFunction(
            self.clientID,
            "Left_Motor",
            sim.sim_scripttype_childscript,
            "moveWheelsByTime",
            [left_speed, right_speed],
            [millis / 1000.0],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )

    def reset_wheels(self) -> None:
        """Allows to reset the wheel encoder positions to 0.
        After calling this topic both encoders (topic /robot/wheels) will start again
        in position 0.
        """
        sim.simxCallScriptFunction(
            self.clientID,
            "Left_Motor",
            sim.sim_scripttype_childscript,
            "resetWheelEncoders",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self.clientID)

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
        sim.simxCallScriptFunction(
            self.clientID,
            "Back_L",
            sim.sim_scripttype_childscript,
            "setLEDColor",
            [],
            [],
            [selector.value, color.value],
            bytearray(),
            sim.simx_opmode_blocking,
        )

    def read_irs(self) -> List[Optional[float]]:
        """Returns sensor readings:
        [backR, backC, backL, frontRR, frontR, frontC, frontL, frontLL]
        """
        ints, _floats, _strings, _buffer = sim.simxCallScriptFunction(
            self.clientID,
            "IR_Back_C",
            sim.sim_scripttype_childscript,
            "readAllIRSensor",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self.clientID)
        return list(ints)

    def get_image_front(self) -> NDArray[numpy.uint8]:
        """Get the image from the front camera as a numpy array in cv2 format.

        You can, for example, write this image to file with:
        https://docs.opencv.org/3.4/d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce
        """
        ping.ping(self.clientID)
        ints, _floats, _strings, buffer = sim.simxCallScriptFunction(
            self.clientID,
            "Smartphone_Respondable",
            sim.sim_scripttype_childscript,
            "getCameraImage",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )

        # reshape image
        image = buffer[::-1]
        im_cv2 = numpy.array(image, dtype=numpy.uint8)
        im_cv2.resize([ints[0], ints[1], 3])
        im_cv2 = cv2.flip(im_cv2, 1)

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
        sim.simxCallScriptFunction(
            self.clientID,
            "Pan_Motor",
            sim.sim_scripttype_childscript,
            "movePanTo",
            [pan_position, pan_speed],
            [],
            [],
            bytearray(),
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
        ints, _floats, _strings, _buffer = sim.simxCallScriptFunction(
            self.clientID,
            "Pan_Motor",
            sim.sim_scripttype_childscript,
            "readPanPosition",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self.clientID)
        return int(ints[0])

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
        sim.simxCallScriptFunction(
            self.clientID,
            "Tilt_Motor",
            sim.sim_scripttype_childscript,
            "moveTiltTo",
            [tilt_position, tilt_speed],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )

    def read_phone_tilt(self) -> int:
        """Get the current tilt of the phone. Range: 26-109"""
        ints, _floats, _strings, _buffer = sim.simxCallScriptFunction(
            self.clientID,
            "Tilt_Motor",
            sim.sim_scripttype_childscript,
            "readTiltPosition",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self.clientID)
        return int(ints[0])

    def read_accel(self) -> Acceleration:
        """Get the acceleration of the robot"""
        _ints, floats, _strings, _buffer = sim.simxCallScriptFunction(
            self.clientID,
            "Smartphone_Respondable",
            sim.sim_scripttype_childscript,
            "readAccelerationSensor",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self.clientID)
        return Acceleration(*floats)

    def read_orientation(self) -> Orientation:
        """Get the orientation of the robot"""
        _ints, floats, _strings, _buffer = sim.simxCallScriptFunction(
            self.clientID,
            "Smartphone_Respondable",
            sim.sim_scripttype_childscript,
            "readOrientationSensor",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self.clientID)
        return Orientation(*floats)

    def read_wheels(self) -> WheelPosition:
        """Get the wheel orientation and speed of the robot"""
        ints, _floats, _strings, _buffer = sim.simxCallScriptFunction(
            self.clientID,
            "Left_Motor",
            sim.sim_scripttype_childscript,
            "readWheels",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self.clientID)
        return WheelPosition(*ints)

    def sleep(self, seconds: float) -> None:
        """Block for a an amount of seconds.
        How to do this depends on the kind of robot, and so is to be found here.
        """
        duration = seconds * 1000
        start_time = self.get_sim_time()
        while self.get_sim_time() - start_time < duration:
            time.sleep(0.02)

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
        # CoppeliaSim ignores blockids - 0 is just there to make the types check out.
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
        return bool(res)

    def block(self):
        """Block untill (only return once) all blocking actions are completed"""
        while self.is_blocked(0):
            time.sleep(0.02)
        ping.ping(self.clientID)

    def play_simulation(self):
        """Start the simulation

        Notice that this can only be called after `pause` and `stop`,
        Calling at the start does not work, as then the API is not initialised jet.
        To start the simulation when CoppeliaSim is started imediately, view the docs here:
        https://www.coppeliarobotics.com/helpFiles/en/commandLine.htm
        """
        sim.simxStartSimulation(self.clientID, simConst.simx_opmode_blocking)
        ping.ping(self.clientID)

    def pause_simulation(self):
        """Pause the simulation"""
        sim.simxPauseSimulation(self.clientID, simConst.simx_opmode_blocking)
        ping.ping(self.clientID)

    def stop_simulation(self):
        """Stop the simulation"""
        sim.simxStopSimulation(self.clientID, simConst.simx_opmode_blocking)
        ping.ping(self.clientID)

    def is_running(self) -> bool:
        """Return wether the simulation is currently running"""
        return not self.get_sim_time() == 0

    def get_sim_time(self) -> int:
        """Get simulation time (in ms), or 0 if the simulation is not running"""
        ping.ping(self.clientID)
        return sim.simxGetLastCmdTime(self.clientID)

    def nr_food_collected(self) -> int:
        """Return the amount of food currently collected.

        Trivially doesn't work when the simulation does not have any food.
        """
        ping.ping(self.clientID)
        ints, _floats, _strings, _buffer = sim.simxCallScriptFunction(
            self.clientID,
            "Food",
            simConst.sim_scripttype_childscript,
            "remote_get_collected_food",
            [],
            [],
            [],
            bytearray(),
            simConst.simx_opmode_blocking,
        )
        return ints[0]
