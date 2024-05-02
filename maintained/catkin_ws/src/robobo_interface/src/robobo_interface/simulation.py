import os
import sys
import time
import signal

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

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from typing import Callable, List, NoReturn, Optional, TypeVar
from numpy.typing import NDArray

T = TypeVar("T")


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
    timeout_dur: int -> The amount of time to wait for API calls before erroring out.
    """

    def __init__(
        self,
        identifier: int = 0,
        api_port: Optional[int] = None,
        ip_adress: Optional[str] = None,
        logger: Callable[[str], None] = print,
        timeout_dur: int = 10,
    ):
        self._logger = logger
        self._used_pids: LockedSet[int] = LockedSet()
        self._identifier = f"[{identifier}]"

        if api_port is None:
            api_port = int(os.getenv("COPPELIA_SIM_PORT", "23000"))

        # 0.0.0.0 to connect to the current computer on Linux, with `--net=host`
        # This doesn't work on Windows or MacOS. There, the variable needs to be specified.
        if ip_adress is None:
            ip_adress = os.getenv("COPPELIA_SIM_IP", "0.0.0.0")

        # The RemoteAPIClient waits indefinetly, but I want some way to show an error.
        # It closes the connection when it gets garbage collected, so no need to close
        try:
            self._client = timeout(
                lambda: RemoteAPIClient(host=ip_adress, port=api_port), timeout_dur
            )
        except TimeoutError:
            self._fail_connect(api_port, ip_adress)

        try:
            self._sim = timeout(lambda: self._client.require("sim"), timeout_dur)
        except TimeoutError:
            self._fail_connect(api_port, ip_adress)

        self._initialise_handles()
        self._logger(
            f"""Connected to remote CoppeliaSim API server at port {api_port}
            Connected to robot: {self._identifier}"""
        )

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
        if not self.is_running():
            raise RuntimeError("Cannot move wheels when simulation is not running")
        if blockid in self._used_pids:
            raise ValueError(f"BlockID {blockid} is already in use: {self._used_pids}")
        blockid = blockid if blockid is not None else self._first_unblocked()
        self._used_pids.add(blockid)

        self._sim.callScriptFunction(
            "moveWheelsByTime",
            self._wheels_script,
            [right_speed, left_speed],
            [millis / 1000.0],
            [self._block_string(blockid)],
            bytearray(),
        )

        return blockid

    def reset_wheels(self) -> None:
        """Allows to reset the wheel encoder positions to 0.
        After calling this topic both encoders (topic /robot/wheels) will start again
        in position 0.
        """
        if not self.is_running():
            raise RuntimeError("Cannot reset wheels when simulation is not running")
        self._sim.callScriptFunction(
            "resetWheelEncoders",
            self._wheels_script,
            [],
            [],
            [],
            bytearray(),
        )

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
        if not self.is_running():
            raise RuntimeError("Cannot set leds when simulation is not running")
        self._sim.callScriptFunction(
            "setLEDColor",
            self._leds_script,
            [],
            [],
            [selector.value, color.value],
            bytearray(),
        )

    def read_irs(self) -> List[Optional[float]]:
        """Returns sensor readings:
        [BackL, BackR, FrontL, FrontR, FrontC, FrontRR, BackC, FrontLL]
        """
        ints, _floats, _strings, _buffer = self._sim.callScriptFunction(
            "readAllIRSensor",
            self._ir_script,
            [],
            [],
            [],
            bytearray(),
        )
        return list(ints)

    def get_image_front(self) -> NDArray[numpy.uint8]:
        """Get the image from the front camera as a numpy array in cv2 format.

        You can, for example, write this image to file with:
        https://docs.opencv.org/3.4/d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce
        """
        img, [resX, resY] = self._sim.getVisionSensorImg(self._smartphone_camera)
        img = numpy.frombuffer(img, dtype=numpy.uint8).reshape(resY, resX, 3)

        # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
        # and color format is RGB triplets, whereas OpenCV uses BGR:
        cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
        return img

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
        if not self.is_running():
            raise RuntimeError("Cannot set phone pan when simulation is not running")
        if blockid in self._used_pids:
            raise ValueError(f"BlockID {blockid} is already in use: {self._used_pids}")
        blockid = blockid if blockid is not None else self._first_unblocked()
        self._used_pids.add(blockid)

        self._sim.callScriptFunction(
            "movePanTo",
            self._pan_motor_script,
            [pan_position, pan_speed],
            [],
            [self._block_string(blockid)],
            bytearray(),
        )

        return blockid

    def read_phone_pan(self) -> int:
        """Get the current pan of the phone. Range: 0-100"""
        ints, _floats, _strings, _buffer = self._sim.callScriptFunction(
            "readPanPosition",
            self._pan_motor_script,
            [],
            [],
            [],
            bytearray(),
        )
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
        if not self.is_running():
            raise RuntimeError("Cannot set phone tilt when simulation is not running")
        if blockid in self._used_pids:
            raise ValueError(f"BlockID {blockid} is already in use: {self._used_pids}")
        blockid = blockid if blockid is not None else self._first_unblocked()
        self._used_pids.add(blockid)

        self._sim.callScriptFunction(
            "moveTiltTo",
            self._tilt_motor_script,
            [tilt_position, tilt_speed],
            [],
            [self._block_string(blockid)],
            bytearray(),
        )

        return blockid

    def read_phone_tilt(self) -> int:
        """Get the current tilt of the phone. Range: 26-109"""
        ints, _floats, _strings, _buffer = self._sim.callScriptFunction(
            "readTiltPosition",
            self._tilt_motor_script,
            [],
            [],
            [],
            bytearray(),
        )

        return int(ints[0])

    def read_accel(self) -> Acceleration:
        """Get the acceleration of the robot"""
        _ints, floats, _strings, _buffer = self._sim.callScriptFunction(
            "readAccelerationSensor",
            self._smartphone_script,
            [],
            [],
            [],
            bytearray(),
        )
        return Acceleration(*floats)

    def read_orientation(self) -> Orientation:
        """Get the orientation of the robot"""
        _ints, floats, _strings, _buffer = self._sim.callScriptFunction(
            "readOrientationSensor",
            self._smartphone_script,
            [],
            [],
            [],
            bytearray(),
        )
        return Orientation(*floats)

    def read_wheels(self) -> WheelPosition:
        """Get the wheel orientation and speed of the robot"""
        ints, _floats, _strings, _buffer = self._sim.callScriptFunction(
            "readWheels",
            self._wheels_script,
            [],
            [],
            [],
            bytearray(),
        )
        return WheelPosition(*ints)

    def sleep(self, seconds: float) -> None:
        """Block for a an amount of seconds.
        How to do this depends on the kind of robot, and so is to be found here.
        """
        start_time = self.get_sim_time()
        while self.get_sim_time() - start_time < seconds:
            if not self.is_running():
                raise RuntimeError("Cannot sleep when simulation is not running")
            time.sleep(0.02)

    def is_blocked(self, blockid: int) -> bool:
        """See if the robot is currently "blocked", which is to say, performing an action

        Arguments:
        blockid: the id to check
        """
        res = self._sim.getInt32Signal(self._block_string(blockid))
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

    def play_simulation(self):
        """Start the simulation"""
        self._sim.startSimulation()

    def pause_simulation(self):
        """Pause the simulation"""
        self._sim.pauseSimulation()
        while not self.is_paused():
            time.sleep(0.002)

    def stop_simulation(self):
        """Stop the simulation"""
        self._sim.stopSimulation()
        while not self.is_stopped():
            time.sleep(0.002)

    def is_stopped(self) -> bool:
        """Return wether the simulation is stopped"""
        return self._sim.getSimulationState() == self._sim.simulation_stopped

    def is_paused(self) -> bool:
        """Return wether the simulation is stopped"""
        return self._sim.getSimulationState() == self._sim.simulation_paused

    def is_running(self) -> bool:
        """Return wether the simulation is running"""
        # There are 6 different types of running we don't care about
        state = self._sim.getSimulationState()
        return (state != self._sim.simulation_stopped) and (
            state != self._sim.simulation_paused
        )

    def get_sim_time(self) -> float:
        """Get simulation time (in seconds),
        returning the last value if the simulation is stopped
        """
        return self._sim.getSimulationTime()

    def nr_food_collected(self) -> int:
        """Return the amount of food currently collected.

        This only works in the simulation
        Trivially doesn't work when the simulation does not have any food.
        """
        ints, _floats, _strings, _buffer = self._sim.callScriptFunction(
            "remote_get_collected_food",
            self._food_script,
            [],
            [],
            [],
            bytearray(),
        )
        return ints[0]

    def get_position(self) -> Position:
        """Get the position of the Robobo (Relative to the world)

        This only works in the simulation.
        """
        pos = self._sim.getObjectPosition(self._robobo, self._sim.handle_world)
        return Position(*pos)

    def get_orientation(self) -> Orientation:
        """Get the orientation of the Robobo (Relative to the world)

        This only works in the simulation.
        """
        orient = self._sim.getOBjectOrientation(self._robobo, self._sim.handle_world)
        return Orientation(*orient)

    def set_position(self, position: Position, orientation: Orientation) -> None:
        """Set the position of the Robobo in the simulation"""
        self._sim.setObjectPosition(
            self._robobo, [position.x, position.y, position.z], self._sim.handle_world
        )
        self._sim.setObjectOrientation(
            self._robobo,
            [orientation.yaw, orientation.pitch, orientation.roll],
            self._sim.handle_world,
        )

    def base_position(self) -> Position:
        """Get the position of the base to deliver food at.

        This only works in the simulation.
        Trivially doesn't work when the simulation does not have a base.
        """
        if self._base is None:
            raise AttributeError("Scene does not have a base")

        pos = self._sim.getObjectPosition(self._base, self._sim.handle_world)
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
        if self._base is None:
            raise AttributeError("Scene does not have a base")

        if self._food_script is None:
            raise AttributeError("Cannot find any food in the scene")

        _ints, floats, _strings, _buffer = self._sim.callScriptFunction(
            "getFoodDistance",
            self._base_script,
            [],
            [],
            [],
            bytearray(),
        )
        ret = floats[0]
        if ret < 0:
            raise AttributeError("Cannot find any food in the scene")
        return ret

    def _block_string(self, blockid: int) -> str:
        """Return some unique string based on the identifier and the blockid
        to make sure they don't overlap
        """
        return f"Block_{self._identifier}_{blockid}"

    def _initialise_handles(self) -> None:
        # fmt: off
        self._robobo = self._get_object(f"/Robobo{self._identifier}")
        self._wheels_script = self._get_childscript(self._get_object(f"/Robobo{self._identifier}/Left_Motor"))
        self._leds_script = self._get_childscript(self._get_object(f"/Robobo{self._identifier}/Back_L"))
        self._ir_script = self._get_childscript(self._get_object(f"/Robobo{self._identifier}/IR_Back_C"))
        self._pan_motor_script = self._get_childscript(self._get_object(f"/Robobo{self._identifier}/Pan_Motor"))
        self._tilt_motor_script = self._get_childscript(self._get_object(f"/Robobo{self._identifier}/Pan_Motor/Pan_Respondable/Tilt_Motor"))
        self._smartphone_script = self._get_childscript(self._get_object(f"/Robobo{self._identifier}/Pan_Motor/Pan_Respondable/Tilt_Motor/Smartphone_Respondable"))
        self._smartphone_camera = self._get_object(f"/Robobo{self._identifier}/Pan_Motor/Pan_Respondable/Tilt_Motor/Smartphone_Respondable/Smartphone_camera")
        # fmt: on

        try:
            self._base = self._get_object("/Base")
            self._base_script = self._get_childscript(self._base)
        except AttributeError:
            self._base = None
            self._base_script = None

        try:
            self._food_script = self._get_childscript(self._get_object("/Food"))
        except AttributeError:
            self._food_script = None

    def _get_object(self, name: str) -> int:
        # CoppeliaSim is a mess sometimes.
        try:
            ret = self._sim.getObject(name)
        except:
            raise AttributeError(f"Could not find {str} in scene")
        if ret < 0:
            raise AttributeError(f"Could not find {str} in scene")
        return ret

    def _get_childscript(self, obj_handle: int) -> int:
        try:
            ret = self._sim.getScript(self._sim.scripttype_childscript, obj_handle)
        except:
            raise AttributeError(f"Could not find Script of {str} in scene")
        if ret < 0:
            raise AttributeError(f"Could not find Script of {str} in scene")
        return ret

    def _fail_connect(self, api_port: int, ip_adress: str) -> NoReturn:
        self._logger(
            """CoppeliaSim Api Connection Error
            Failed connecting to remote API server
            Is the simulation running / playing?

            If not on Linux with --net=host:
            Did you specify the IP adress of your computer in scripts/setup.bash?
            """
        )
        self._logger(f"Looked for API at port: {api_port} at IP adress: {ip_adress}")
        # Yes, sys.exit(1) gets caught by the zmq runtime. No, I don't know why.
        quit_hard()


# This only works on Unix. Luckily, we are in Docker.
def timeout(func: Callable[[], T], timeout_duration: int = 10) -> T:
    def handler(_signum, _frame):
        raise TimeoutError()

    # set the timeout handler
    signal.signal(signal.SIGALRM, handler)
    signal.alarm(timeout_duration)
    try:
        return func()
    finally:
        signal.alarm(0)


# The API code catches too much, making it hard to quit when failing.
# This is about as agressive as it gets, and should not be used except in containers.
def quit_hard() -> NoReturn:
    os.kill(os.getpid(), signal.SIGKILL)
    # The above is what does it, but the below is to make the type system happy
    sys.exit(1)
