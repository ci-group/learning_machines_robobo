import os

import rospy
import cv2
import numpy
from std_msgs.msg import String, Int8, Int16, Int32
from geometry_msgs.msg import Accel
from sensor_msgs.msg import CompressedImage
from robobo_msgs.srv import (
    MoveWheels,
    MovePanTilt,
    SetEmotion,
    PlaySound,
    Talk,
    SetLed,
    ResetWheels,
)
from robobo_msgs.msg import IRs, Wheels, OrientationEuler

from robobo_interface.base import IRobobo
from robobo_interface.datatypes import (
    Emotion,
    LedColor,
    LedId,
    Acceleration,
    Orientation,
    SoundEmotion,
    WheelPosition,
)
from robobo_interface.utils import LockedSet

from typing import Callable, List, Optional
from numpy.typing import NDArray

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Services#robotmovewheels
MOVE_WHEELS_SERVICE = "robot/moveWheels"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Services#robotmovepantilt
PAN_TILT_SERVICE = "robot/movePanTilt"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Services#robotsetemotion
SET_EMOTION_SERVICE = "robot/setEmotion"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Services#robotplaysound
PLAY_EMOTION_SERVICE = "robot/playSound"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Services#robottalk
TALK_SERVICE = "robot/talk"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Services#robotsetled
SET_LED_SERVICE = "robot/setLed"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotcameraimagecompressed
IMAGE_TOPIC = "robot/camera/image/compressed"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Services#robotsetcamera
SET_CAMERA_SERVICE = "robot/setCamera"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotirs
IRS_TOPIC = "robot/irs"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotreset_wheels
WHEEL_RESET_SERVICE = "robot/resetWheels"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotpan
PAN_TOPIC = "robot/pan"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robottilt
TILT_TOPIC = "robot/tilt"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotaccel
ACCEL_TOPIC = "robot/accel"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotorientation_euler
ORIENTATION_TOPIC = "robot/orientation_euler"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotwheels
WHEEL_TOPIC = "robot/wheels"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotunlockmove
UNLOCK_MOVE_TOPIC = "/robot/unlock/move"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotbatterybase
ROBOT_BATTERY_TOPIC = "robot/battery/base"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotbatteryphone
PHONE_BATTERY_TOPIC = "robot/battery/phone"


class HardwareRobobo(IRobobo):
    """The class to use to interact with the hardware Robobo
    Implements the IRobobo interface, and shouldn't really be used outside of that.

    Specific hardware documentation of the robobo can be found at the scratch part of the Robobo docs:
    - Actuation (Robot): http://education.theroboboproject.com/en/scratch3/base-actuation-blocks
    - Sensors (Robot): http://education.theroboboproject.com/en/scratch3/base-sensing-blocks
    - Actuation (Smartphone): http://education.theroboboproject.com/en/scratch3/smartphone-actuation-blocks
    - Sensors (Smartphone): http://education.theroboboproject.com/en/scratch3/smartphone-sensing-blocks

    On the hardware, read operations only update when they change, meaning that any value will
    be 0 if it didn't change since calling initialising this class.

    Blocking on the Robobo itself behaves differently than blocking on the simulation.
    Any new call to one particular service (which is to say, a function that takes a `blockid` as argument)
    will immediately cancel a previous unfished order.
    So, for example, if we call `rob.move` when a previous movement command is not finished,
    the Robobo will start executing the new command, thus changing the movement without stopping.
    https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Services#robobo-services

    It is for this reason it is recommended to only use the `_blocking` functions of the robot,
    which are inherited from the IRobobo in the format of a template method.

    However, if you want the robot to move the tilt motor while also driving and such,
    you can still experiment with them.

    Arguments you should understand:
    camera: bool = False -> Wether to initialise the camera (this makes it slower)

    Arguments you only have to understand if you want to do advanced stuff:
    xmlrpc_port: Optional[int] = None -> The port to start the xmlrps of the ROS node at.
        If None, it will try to see if ROS_XMLRPC_PORT is set, and use that.
        If that envoirement variable is not set, it will default to 45100
    tcpros_port: Optional[int] = None -> The port to start the tcpros of the ROS node at.
        If None, it will try to see if ROS_XMLRPC_PORT is set, and use that.
        If that envoirement variable is not set, it will default to 45101
    logger: Callable[[str], None] = print -> The function to use for logging / printing.
    """

    def __init__(
        self,
        camera=False,
        xmlrpc_port: Optional[int] = None,
        tcpros_port: Optional[int] = None,
        logger: Callable[[str], None] = rospy.loginfo,
    ) -> None:
        """This sets up the HardwareRobobo, and presumes everything is running
        So, it cannot be started up unless the Robobo is actually connected.

        Arguments
        camera: bool - Whether or not to enable the camera
        """
        self._logger = logger
        self._enable_camera: bool = camera

        if xmlrpc_port is None:
            xmlrpc_port = int(os.getenv("ROS_XMLRPC_PORT", "45100"))
        if tcpros_port is None:
            tcpros_port = int(os.getenv("ROS_TCPROS_PORT", "45101"))

        rospy.init_node(
            "learning_machines_robobo_controler",
            xmlrpc_port=xmlrpc_port,
            tcpros_port=tcpros_port,
        )
        self._logger("Starting the Learning Machines robobo controller node")

        # Service Proxys
        self._move_srv = rospy.ServiceProxy(MOVE_WHEELS_SERVICE, MoveWheels)
        self._reset_wheels_src = rospy.ServiceProxy(WHEEL_RESET_SERVICE, ResetWheels)
        self._pan_tilt_srv = rospy.ServiceProxy(PAN_TILT_SERVICE, MovePanTilt)
        self._emotion_srv = rospy.ServiceProxy(SET_EMOTION_SERVICE, SetEmotion)
        self._sound_emotion_srv = rospy.ServiceProxy(PLAY_EMOTION_SERVICE, PlaySound)
        self._talk_srv = rospy.ServiceProxy(TALK_SERVICE, Talk)
        self._leds_srv = rospy.ServiceProxy(SET_LED_SERVICE, SetLed)

        # locking
        self._used_pids: LockedSet[int] = LockedSet()
        self._mvunlocksub = rospy.Subscriber(
            UNLOCK_MOVE_TOPIC, Int16, self._unlock_move_callback
        )

        # Battery indicators are nice to have
        self._robot_battery_val = 100.0
        self._robot_battery_sub = rospy.Subscriber(
            ROBOT_BATTERY_TOPIC, Int8, self._robot_battery_callback
        )
        self._phone_battery_val = 100.0
        self._phone_battery_sub = rospy.Subscriber(
            PHONE_BATTERY_TOPIC, Int8, self._phone_battery_callback
        )

        # Sensor Receivers
        self._irs_values = [0.0 for _ in range(8)]
        self._irsub = rospy.Subscriber(IRS_TOPIC, IRs, self._irs_callback)

        self._pan = 0
        self._pansub = rospy.Subscriber(PAN_TOPIC, Int16, self._pan_callback)

        self._tilt = 0
        self._tiltsub = rospy.Subscriber(TILT_TOPIC, Int16, self._tilt_callback)

        self._accel = Acceleration()
        self._accelsub = rospy.Subscriber(ACCEL_TOPIC, Accel, self._accel_callback)

        self._orient = Orientation()
        self._orientsub = rospy.Subscriber(
            ORIENTATION_TOPIC, OrientationEuler, self._orient_callback
        )

        self._wheelpos = WheelPosition()
        self._wheelsub = rospy.Subscriber(WHEEL_TOPIC, Wheels, self._wheelpos_callback)

        if self._enable_camera:
            self._receiving_image_front = None
            self._image_subscribe_front = rospy.Subscriber(
                IMAGE_TOPIC, CompressedImage, self._camera_callback_front, queue_size=1
            )

        self._logger("Succesfully initialised Learning Machines robobo controller node")

    def set_emotion(self, emotion: Emotion) -> None:
        """Show the emotion of the robot.
        For the hardware, this means showing on the phone screen.

        Arguments

        emotion: Emotion - What emotion to show.
        """
        self._emotion_srv(String(emotion.value))

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
        blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic.
            Use a value that is within a 16-bit integer limit
            If None is passed, a random available blockid is chosen.

        returns:
            the blockid
        """
        if blockid in self._used_pids:
            raise ValueError(f"BlockID {blockid} is already in use: {self._used_pids}")
        blockid = blockid if blockid is not None else self._first_unblocked()
        self._used_pids.add(blockid)
        self._move_srv(
            Int8(left_speed), Int8(right_speed), Int32(millis), Int16(blockid)
        )
        return blockid

    def reset_wheels(self) -> None:
        """Allows to reset the wheel encoder positions to 0.
        After calling this topic both encoders (topic /robot/wheels) will start again
        in position 0.
        """
        self._reset_wheels_src()

    def talk(self, message: str) -> None:
        """Let the robot speak.

        Arguments
        message: str - what to say
        """
        self._talk_srv(String(message))

    def play_emotion_sound(self, emotion: SoundEmotion) -> None:
        """Let the robot make an emotion sound

        Arguments:
        emotion: SoundEmotion - The sound to make.
        """
        self._sound_emotion_srv(String(emotion.value))

    def set_led(self, selector: LedId, color: LedColor) -> None:
        """Set the led of the robot

        Arguments:
        selector: LedId
        color: LedColor
        """
        self._leds_srv(String(selector.value), String(color.value))

    def read_irs(self) -> List[Optional[float]]:
        """Returns sensor readings:
        [BackL, BackR, FrontL, FrontR, FrontC, FrontRR, BackC, FrontLL]
        """
        return self._irs_values

    def get_image_front(self) -> NDArray[numpy.uint8]:
        """Get the image from the front camera as a numpy array in cv2 format.

        You can, for example, write this image to file with:
        https://docs.opencv.org/3.4/d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce
        """
        if not self._enable_camera:
            raise ValueError("Camera is disabled")

        # Retrieve last image from image topic
        self._receiving_image_front = None
        while self._receiving_image_front is None:
            self.sleep(0.002)
        image = self._receiving_image_front
        self._receiving_image_front = None
        return image

    def set_phone_pan(
        self, pan_position: int, pan_speed: int, blockid: Optional[int] = None
    ) -> int:
        """Command the robot to move the smartphone holder in the horizontal (pan) axis.

        Notice that the robot, especially on high speeds, doesn't always run perfectly

        Arguments
        pan_position: Angle to position the pan at. Range: 11-343.
            Centre appears to be around 123
            This is the goal, the result might be sloppy, and should be checked.
        pan_speed: Movement speed for the pan mechanism. Range: 0-100.
        blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic.
            Use a value that is within a 16-bit integer limit
            If None is passed, a random available blockid is chosen.

        returns:
            the blockid
        """
        if blockid in self._used_pids:
            raise ValueError(f"BlockID {blockid} is already in use: {self._used_pids}")
        blockid = blockid if blockid is not None else self._first_unblocked()
        self._used_pids.add(blockid)
        self._pan_tilt_srv(
            Int16(pan_position),
            Int8(pan_speed),
            Int16(blockid),
            Int16(0),
            Int8(0),
            Int16(0),
        )
        return blockid

    def read_phone_pan(self) -> int:
        """Get the current pan of the phone. Range: 0-100

        For the hardware: This isn't guaranteed to be the same calibration as the movement,
        but unlikely to be more than one or two units off
        """
        return self._pan

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
        if blockid in self._used_pids:
            raise ValueError(f"BlockID {blockid} is already in use: {self._used_pids}")
        blockid = blockid if blockid is not None else self._first_unblocked()
        self._used_pids.add(blockid)
        self._pan_tilt_srv(
            Int16(0),
            Int8(0),
            Int16(0),
            Int16(tilt_position),
            Int8(tilt_speed),
            Int16(blockid),
        )
        return blockid

    def read_phone_tilt(self) -> int:
        """Get the current tilt of the phone. Range: 26-109

        For the hardware: This isn't guaranteed to be the same calibration as the movement,
        but unlikely to be more than one or two units off
        """
        return self._tilt

    def read_accel(self) -> Acceleration:
        """Get the acceleration of the robot"""
        return self._accel

    def read_orientation(self) -> Orientation:
        """Get the orientation of the robot"""
        return self._orient

    def read_wheels(self) -> WheelPosition:
        """Get the wheel orientation and speed of the robot"""
        return self._wheelpos

    def read_phone_battery(self) -> float:
        """Get the battety percentage of the phone

        Note that the battery nodes don't send their data frequently,
        this might be the default value (100) if you call it in the first minute

        If the battery is below 10%, this code will log it automatically,
        so no need to implement that.
        """
        return self._phone_battery_val

    def read_robot_battery(self) -> float:
        """Get the battety percentage of the robot

        Note that the battery nodes don't send their data frequently,
        this might be the default value (100) if you call it in the first minute

        If the battery is below 10%, this code will log it automatically,
        so no need to implement that.
        """
        return self._robot_battery_val

    def sleep(self, seconds: float) -> None:
        """Block for a an amount of seconds.
        How to do this depends on the kind of robot, and so is to be found here.
        """
        rospy.sleep(seconds)

    def is_blocked(self, blockid: int) -> bool:
        """See if the robot is currently "blocked", which is to say, performing an action

        Arguments:
        blockid: the id to check
        """
        return blockid in self._used_pids

    def block(self) -> None:
        """Block untill (e.g. only return once) all blocking actions are completed"""
        while len(self._used_pids):
            self.sleep(0.002)

    def _irs_callback(self, ros_data: IRs) -> None:
        self._irs_values = [
            ros_data.BackL.range,
            ros_data.BackR.range,
            ros_data.FrontL.range,
            ros_data.FrontR.range,
            ros_data.FrontC.range,
            ros_data.FrontRR.range,
            ros_data.BackC.range,
            ros_data.FrontLL.range,
        ]

    def _camera_callback_front(self, ros_data: CompressedImage):
        if self._receiving_image_front is None:
            np_arr = numpy.fromstring(ros_data.data, numpy.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self._receiving_image_front = cv2.flip(image_np, 1)

    def _pan_callback(self, ros_data: Int16) -> None:
        self._pan = ros_data.data

    def _tilt_callback(self, ros_data: Int16) -> None:
        self._tilt = ros_data.data

    def _accel_callback(self, ros_data: Accel) -> None:
        self._accel = Acceleration(
            x=ros_data.linear.x,
            y=ros_data.linear.y,
            z=ros_data.linear.z,
        )

    def _orient_callback(self, ros_data: OrientationEuler) -> None:
        self._orient = Orientation(
            yaw=ros_data.yaw.data,
            pitch=ros_data.pitch.data,
            roll=ros_data.roll.data,
        )

    def _wheelpos_callback(self, ros_data: Wheels) -> None:
        self._wheelpos = WheelPosition(
            wheel_pos_r=ros_data.wheelPosR.data,
            wheel_pos_l=ros_data.wheelPosL.data,
            wheel_speed_r=ros_data.wheelSpeedR.data,
            wheel_speed_l=ros_data.wheelSpeedL.data,
        )

    def _unlock_move_callback(self, ros_data: Int16) -> None:
        self._used_pids.discard(ros_data.data)

    def _phone_battery_callback(self, ros_data: Int8) -> None:
        self._phone_battery_val = ros_data.data
        if ros_data.data < 10:
            self._logger(f"Phone battery is getting low: {ros_data.data}%")

    def _robot_battery_callback(self, ros_data: Int8) -> None:
        self._robot_battery_val = ros_data.data
        if ros_data.data < 10:
            self._logger(f"Robot battery is getting low: {ros_data.data}%")
