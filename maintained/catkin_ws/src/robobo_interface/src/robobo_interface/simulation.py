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
    Position,
    Orientation,
    WheelPosition,
    SoundEmotion,
)
from robobo_interface.utils import LockedSet

from coppelia_sim import sim, simConst, ping
from coppelia_sim.error import CoppeliaSimApiError

from typing import Callable, List, Optional
from numpy.typing import NDArray


class SimulationRobobo(IRobobo):
    """The simulation robot.

    A few functions take blockid. When they do, they are not blocking. If you call
    these while the action is still blocked with a new blockid
    stuff might go seriously wrong, and the robot might halt indefinetly.

    Since the behavior of non-blocking functions is different from Simulation to Hardware,
    it is recommended to only use the `_blocking` functions of the robot,
    which are inherited from the IRobobo in the format of a template method.

    However, if you want the robot to move the tilt motor while also driving and such,
    you can still experiment with them.

    Arguments you should understand:
    realtime: bool = False -> Wether to run the simulation in realtime, or as fast as possible
    identifier: int = 0 -> The value number to use. If you have one robobo in the scene,
            or want to use the first one, this is 0. Else, increase the value

    Arguments you only have to understand if you want to do advanced stuff:
    api_port: Optional[int] = None -> The port at which to look for the CoppeliaSim API.
        If None, it will try to see if COPPELIA_SIM_PORT is set, and use that.
        If that envoirement variable is not set, it will default to 19999
    ip_adress: Optional[str] = None -> The TCP adress at which to look for the CoppeliaSim API.
        If None, it will try to see if COPPELIA_SIM_IP is set, and use that.
        If that envoirement variable is not set, it will default to "0.0.0.0"
    logger: Callable[[str], None] = print -> The function to use for logging / printing.
    """

    def __init__(
        self,
        realtime=False,
        identifier: int = 0,
        api_port: Optional[int] = None,
        ip_adress: Optional[str] = None,
        logger: Callable[[str], None] = print,
    ):
        self._logger = logger
        self._used_pids: LockedSet[int] = LockedSet()
        self._identifier = f"[{identifier}]"

        sim.simxFinish(-1)  # just in case, close all opened connections
        # 0.0.0.0 to connect to the current computer on Linux, with `--net=host`
        # This doesn't work on Windows or MacOS. There, the variable needs to be specified.

        if api_port is None:
            api_port = int(os.getenv("COPPELIA_SIM_PORT", "19999"))
        if ip_adress is None:
            ip_adress = os.getenv("COPPELIA_SIM_IP", "0.0.0.0")

        self._connection_id = sim.simxStart(ip_adress, api_port, True, True, 5000, 5)
        if self._connection_id == -1:
            self._logger(
                """CoppeliaSim Api Connection Error
                Failed connecting to remote API server
                Is the simulation running / playing?

                If not on Linux with --net=host:
                Did you specify the IP adress of your computer in scripts/setup.bash?
                """
            )
            self._logger(
                f"Looked for API at port: {api_port} at IP adress: {ip_adress}"
            )
            sys.exit(1)

        ping.ping(self._connection_id)
        self._logger(
            f"""Connected to remote CoppeliaSim API server at port {api_port}
            Connected to robot: {self._identifier}"""
        )

        self._initialise_handles()
        self.set_realtime(realtime)

    def __del__(self):
        # Yes, having __enter__ and __exit__ on a seperate CoppeliaSimConnection object would be cleaner.
        # However, this gives the simler API, for which I am willing to sacrifice code quality.
        sim.simxFinish(self._connection_id)

    def set_emotion(self, emotion: Emotion) -> None:
        """Show the emotion of the robot on the screen
        For the hardware, that means printing... for now.
        The one that was in the example from mintforpeople does not work.

        Arguments
        emotion: Emotion - What emotion to show.
        """
        self._logger(f"The robot shows {emotion.value} on its screen")

    def move(
        self,
        left_speed: int,
        right_speed: int,
        millis: int,
        blockid: Optional[int] = None,
    ) -> int:
        """Move the robot wheels for `millis` time

        Arguments
        left_speed: speed of the left wheel. Range: -100-0-100. 0 is no movement, negative backward.
        right_speed: speed of the right wheel. Range: -100-0-100. 0 is no movement, negative backward.
        millis: how many millisecond to move the robot
        blockid: A unique blockid to test if the robot is still perfoming the action.
            If None is passed, a random available blockid is chosen.

        returns:
            the blockid
        """
        if blockid in self._used_pids:
            raise ValueError(f"BlockID {blockid} is already in use: {self._used_pids}")
        blockid = blockid if blockid is not None else self._first_unblocked()
        self._used_pids.add(blockid)

        sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/Left_Motor",
            sim.sim_scripttype_childscript,
            "moveWheelsByTime",
            [right_speed, left_speed],
            [millis / 1000.0],
            [self._block_string(blockid)],
            bytearray(),
            sim.simx_opmode_blocking,
        )

        return blockid

    def reset_wheels(self) -> None:
        """Allows to reset the wheel encoder positions to 0.
        After calling this topic both encoders (topic /robot/wheels) will start again
        in position 0.
        """
        sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/Left_Motor",
            sim.sim_scripttype_childscript,
            "resetWheelEncoders",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self._connection_id)

    def talk(self, message: str) -> None:
        """Let the robot speak.
        For the simultion, this is just printing.

        Arguments
        message: str - what to say
        """
        self._logger(f"The robot {self._identifier} says: {message}")

    def play_emotion_sound(self, emotion: SoundEmotion) -> None:
        """Let the robot make an emotion sound
        For the simultion, this is just printing.

        Arguments:
        emotion: SoundEmotion - The sound to make.
        """
        self._logger(f"The robot {self._identifier} makes sound: {emotion.value}")

    def set_led(self, selector: LedId, color: LedColor) -> None:
        """Set the led of the robot

        Arguments:
        selector: LedId
        color: LedColor
        """
        sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/Back_L",
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
        [BackL, BackR, FrontL, FrontR, FrontC, FrontRR, BackC, FrontLL]
        """
        ints, _floats, _strings, _buffer = sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/IR_Back_C",
            sim.sim_scripttype_childscript,
            "readAllIRSensor",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self._connection_id)
        return list(ints)

    def get_image_front(self) -> NDArray[numpy.uint8]:
        """Get the image from the front camera as a numpy array in cv2 format.

        You can, for example, write this image to file with:
        https://docs.opencv.org/3.4/d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce
        """
        ping.ping(self._connection_id)
        ints, _floats, _strings, buffer = sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/Pan_Motor/Pan_Respondable/Tilt_Motor/Smartphone_Respondable",
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
    ) -> int:
        """Command the robot to move the smartphone holder in the horizontal (pan) axis.
        This function is asyncronous.

        Arguments
        pan_position: Angle to position the pan at. Range: 11-343.
        pan_speed: Movement speed for the pan mechanism. Range: 0-100.
        blockid: A unique blockid to test if the robot is still perfoming the action.
            If None is passed, a random available blockid is chosen.

        returns:
            the blockid
        """
        if blockid in self._used_pids:
            raise ValueError(f"BlockID {blockid} is already in use: {self._used_pids}")
        blockid = blockid if blockid is not None else self._first_unblocked()
        self._used_pids.add(blockid)

        sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/Pan_Motor",
            sim.sim_scripttype_childscript,
            "movePanTo",
            [pan_position, pan_speed],
            [],
            [self._block_string(blockid)],
            bytearray(),
            sim.simx_opmode_blocking,
        )

        return blockid

    def read_phone_pan(self) -> int:
        """Get the current pan of the phone. Range: 0-100"""
        ints, _floats, _strings, _buffer = sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/Pan_Motor",
            sim.sim_scripttype_childscript,
            "readPanPosition",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self._connection_id)
        return int(ints[0])

    def set_phone_tilt(
        self, tilt_position: int, tilt_speed: int, blockid: Optional[int] = None
    ) -> int:
        """Command the robot to move the smartphone holder in the vertical (tilt) axis.
        This function is asyncronous.

        Arguments
        tilt_position: Angle to position the tilt at. Range: 26-109.
        tilt_speed: Movement speed for the tilt mechanism. Range: 0-100.
        blockid: A unique blockid to test if the robot is still perfoming the action.
            If None is passed, a random available blockid is chosen.

        returns:
            the blockid
        """
        if blockid in self._used_pids:
            raise ValueError(f"BlockID {blockid} is already in use: {self._used_pids}")
        blockid = blockid if blockid is not None else self._first_unblocked()
        self._used_pids.add(blockid)

        sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/Pan_Motor/Pan_Respondable/Tilt_Motor",
            sim.sim_scripttype_childscript,
            "moveTiltTo",
            [tilt_position, tilt_speed],
            [],
            [self._block_string(blockid)],
            bytearray(),
            sim.simx_opmode_blocking,
        )

        return blockid

    def read_phone_tilt(self) -> int:
        """Get the current tilt of the phone. Range: 26-109"""
        ints, _floats, _strings, _buffer = sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/Pan_Motor/Pan_Respondable/Tilt_Motor",
            sim.sim_scripttype_childscript,
            "readTiltPosition",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self._connection_id)
        return int(ints[0])

    def read_accel(self) -> Acceleration:
        """Get the acceleration of the robot"""
        _ints, floats, _strings, _buffer = sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/Pan_Motor/Pan_Respondable/Tilt_Motor/Smartphone_Respondable",
            sim.sim_scripttype_childscript,
            "readAccelerationSensor",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self._connection_id)
        return Acceleration(*floats)

    def read_orientation(self) -> Orientation:
        """Get the orientation of the robot"""
        _ints, floats, _strings, _buffer = sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/Pan_Motor/Pan_Respondable/Tilt_Motor/Smartphone_Respondable",
            sim.sim_scripttype_childscript,
            "readOrientationSensor",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self._connection_id)
        return Orientation(*floats)

    def read_wheels(self) -> WheelPosition:
        """Get the wheel orientation and speed of the robot"""
        ints, _floats, _strings, _buffer = sim.simxCallScriptFunction(
            self._connection_id,
            f"/Robobo{self._identifier}/Left_Motor",
            sim.sim_scripttype_childscript,
            "readWheels",
            [],
            [],
            [],
            bytearray(),
            sim.simx_opmode_blocking,
        )
        ping.ping(self._connection_id)
        return WheelPosition(*ints)

    def sleep(self, seconds: float) -> None:
        """Block for a an amount of seconds.
        How to do this depends on the kind of robot, and so is to be found here.
        """
        duration = seconds * 1000
        start_time = self.get_sim_time()
        while self.get_sim_time() - start_time < duration:
            time.sleep(0.02)

    def is_blocked(self, blockid: int) -> bool:
        """See if the robot is currently "blocked", which is to say, performing an action

        Arguments:
        blockid: the id to check
        """
        res = sim.simxGetInt32Signal(
            self._connection_id,
            self._block_string(blockid),
            sim.simx_opmode_blocking,
        )
        if res == 0:
            self._used_pids.discard(blockid)
            return False
        else:
            self._used_pids.add(blockid)
            return True

    def block(self):
        """Block untill (only return once) all blocking actions are completed"""
        while any(self.is_blocked(blockid) for blockid in self._used_pids):
            time.sleep(0.02)
        ping.ping(self._connection_id)

    def play_simulation(self):
        """Start the simulation"""
        sim.simxStartSimulation(self._connection_id, simConst.simx_opmode_blocking)
        ping.ping(self._connection_id)

    def pause_simulation(self):
        """Pause the simulation"""
        sim.simxPauseSimulation(self._connection_id, simConst.simx_opmode_blocking)
        ping.ping(self._connection_id)

    def stop_simulation(self):
        """Stop the simulation"""
        sim.simxStopSimulation(self._connection_id, simConst.simx_opmode_blocking)
        ping.ping(self._connection_id)

    def is_running(self) -> bool:
        """Return wether the simulation is currently running"""
        return not self.get_sim_time() == 0

    def get_sim_time(self) -> int:
        """Get simulation time (in ms), or 0 if the simulation is not running"""
        ping.ping(self._connection_id)
        return sim.simxGetLastCmdTime(self._connection_id)

    def nr_food_collected(self) -> int:
        """Return the amount of food currently collected.

        This only works in the simulation
        Trivially doesn't work when the simulation does not have any food.
        """
        ping.ping(self._connection_id)
        ints, _floats, _strings, _buffer = sim.simxCallScriptFunction(
            self._connection_id,
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

    def set_realtime(self, value: bool = True, /) -> None:
        """Make the simulation run either at the actual speed, or as fast as it can.

        Arguments:
        value: bool = False -> to set realtime true or false
        """
        sim.simxSetBoolParam(
            self._connection_id,
            simConst.sim_boolparam_realtime_simulation,
            value,
            simConst.simx_opmode_oneshot,
        )

    def position(self) -> Position:
        """Get the position of the Robobo

        This only works in the simulation.
        """
        pos = sim.simxGetObjectPosition(
            self._connection_id, self._robobo, -1, simConst.simx_opmode_blocking
        )
        return Position(*pos)

    def set_position(self, position: Position, orientation: Orientation) -> None:
        """Set the position of the Robobo in the simulation"""
        sim.simxSetObjectOrientation(
            self._connection_id,
            self._robobo,
            -1,
            [orientation.yaw, orientation.pitch, orientation.roll],
            simConst.simx_opmode_blocking,
        )
        sim.simxSetObjectPosition(
            self._connection_id,
            self._robobo,
            -1,
            [position.x, position.y, position.z],
            simConst.simx_opmode_blocking,
        )

    def base_position(self) -> Position:
        """Get the position of the base to deliver food at.

        This only works in the simulation.
        Trivially doesn't work when the simulation does not have a base.
        """
        if self._base is None:
            raise ValueError("Connected scene does not appear to have a base")

        pos = sim.simxGetObjectPosition(
            self._connection_id, self._base, -1, simConst.simx_opmode_blocking
        )
        return Position(*pos)

    def base_detects_food(self) -> bool:
        """Get whether the base detects food on top of it.

        This only works in the simulation.
        Trivially doesn't work when the simulation does not have a base or food.
        """
        return self._base_food_distance() > 0

    def _base_food_distance(self) -> float:
        """Get the distance between the food and the base,
        0 if the food is too far away (which is to say, not on the plate)

        This only works in the simulation.
        Trivially doesn't work when the simulation does not have a base or food.
        """
        ping.ping(self._connection_id)
        _ints, floats, _strings, _buffer = sim.simxCallScriptFunction(
            self._connection_id,
            "/Base",
            simConst.sim_scripttype_childscript,
            "getFoodDistance",
            [],
            [],
            [],
            bytearray(),
            simConst.simx_opmode_blocking,
        )
        ret = floats[0]
        if ret < 0:
            raise ValueError("Cannot find any food in the scene")
        return ret

    def _block_string(self, blockid: int) -> str:
        """Return some unique string based on the identifier and the blockid
        to make sure they don't overlap
        """
        return f"Block_{self._identifier}_{blockid}"

    def _initialise_handles(self) -> None:
        self._robobo = sim.simxGetObjectHandle(
            self._connection_id,
            f"/Robobo{self._identifier}",
            simConst.simx_opmode_blocking,
        )

        # The base might not exist in this scene.
        try:
            self._base = sim.simxGetObjectHandle(
                self._connection_id,
                "/Base",
                simConst.simx_opmode_blocking,
            )
        except CoppeliaSimApiError:
            self._base = None
