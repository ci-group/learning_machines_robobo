import os
import time

import rospy
import cv2
import numpy
from std_msgs.msg import String, Int8, Int16, Int32
from geometry_msgs.msg import Accel, Quaternion
from sensor_msgs.msg import CompressedImage
from robobo_msgs.srv import (
    MoveWheels,
    MovePanTilt,
    SetEmotion,
    Talk,
    SetLed,
    ResetWheels,
)
from robobo_msgs.msg import IRs, Wheels

from .base import (
    IRobobo,
    Emotion,
    LedColor,
    LedId,
    Acceleration,
    Orientation,
    WheelPosition,
)

from typing import List, Optional
from numpy.typing import NDArray

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotmove_wheels
MOVE_WHEELS_TOPIC = "robot/moveWheels"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotmove_pan_tilt
PAN_TILT_TOPIC = "robot/movePanTilt"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotemotion
SET_EMOTION_TOPIC = "robot/setEmotion"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robottalk
TALK_TOPIC = "robot/talk"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotset_led
SET_LED_TOPIC = "robot/setLed"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotcameraimagecompressed
IMAGE_TOPIC = "robot/camera/image/compressed"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotirs
IRS_TOPIC = "robot/irs"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotreset_wheels
WHEEL_RESET_TOPIC = "robot/reset_wheels"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotpan
PAN_TOPIC = "robot/pan"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robottilt
TILT_TOPIC = "robot/tilt"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotaccel
ACCEL_TOPIC = "robot/accel"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotorientation
ORIENTATION_TOPIC = "robot/orientation"

# https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotwheels
WHEEL_TOPIC = "robot/wheels"


class HardwareRoboboException(Exception):
    pass


class HardwareRobobo(IRobobo):
    def __init__(self, camera=False) -> None:
        """This sets up the HardwareRobobo, and presumes everything is running
        So, it cannot be started up unless the Robobo is actually connected.

        Arguments
        camera: bool - Whether or not to enable the camera
        """
        self._enable_camera = camera

        rospy.init_node("learning_machines_robobo_controler")
        rospy.loginfo("Starting the Learning Machines robobo controller node")

        # Service Proxys
        self._move_srv = rospy.ServiceProxy(MOVE_WHEELS_TOPIC, MoveWheels)
        self._reset_wheels_src = rospy.ServiceProxy(WHEEL_RESET_TOPIC, ResetWheels)
        self._pan_tilt_srv = rospy.ServiceProxy(PAN_TILT_TOPIC, MovePanTilt)
        self._emotion_srv = rospy.ServiceProxy(SET_EMOTION_TOPIC, SetEmotion)
        self._talk_srv = rospy.ServiceProxy(TALK_TOPIC, Talk)
        self._leds_srv = rospy.ServiceProxy(SET_LED_TOPIC, SetLed)

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
            ORIENTATION_TOPIC, Quaternion, self._orient_callback
        )

        self._wheelpos = WheelPosition
        self._wheelsub = rospy.Subscriber(WHEEL_TOPIC, Wheels, self._wheelpos_callback)

        if self._enable_camera:
            self._receiving_image_front = None
            self._image_subscribe_front = rospy.Subscriber(
                IMAGE_TOPIC, CompressedImage, self._camera_callback_front, queue_size=1
            )

        rospy.loginfo(
            "Succesfully initialised Learning Machines robobo controller node"
        )

    def set_emotion(self, emotion: Emotion) -> None:
        """Show the emotion of the robot.
        For the hardware, this means showing on the phone screen.
        https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotemotion

        Arguments

        emotion: Emotion - What emotion to show.
        """
        self._emotion_srv(String(emotion.value))

    def move(
        self, left_speed: int, right_speed: int, millis: int, blockid: int = 0
    ) -> None:
        """
        Move the robot wheels for `millis` time
        https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotmove_wheels

        Arguments

        left_speed: speed of the left wheel. Range: 0-100
        right_speed: speed of the right wheel. Range: 0-100
        millis: how many millisecond to move the robot
        blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic.
        """
        self._move_srv(
            Int8(left_speed), Int8(right_speed), Int32(millis), Int16(blockid)
        )
        self.sleep(millis / 1000.0)

    def reset_wheels(self) -> None:
        """Allows to reset the wheel encoder positions to 0.
        After calling this topic both encoders (topic /robot/wheels) will start again
        in position 0.
        """
        self._reset_wheels_src(Int8(0))

    def talk(self, message: str) -> None:
        """Let the robot speak.
        https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robottalk

        Arguments

        message: str - what to say
        """
        self._talk_srv(String(message))

    def set_led(self, selector: LedId, color: LedColor) -> None:
        """Set the led of the robot
        https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotset_led

        Arguments:
        selector: LedId
        color: LedColor
        """
        self._leds(String(selector.value), String(color.value))

    def read_irs(self) -> List[Optional[float]]:
        """
        Returns sensor readings: [backR, backC, backL, frontRR, frontR, frontC, frontL, frontLL]
        """
        return self._irs_values

    def get_image_front(self) -> NDArray[numpy.uint8]:
        """Get the image from the front camera as a numpy array"""
        if not self._enable_camera:
            raise HardwareRoboboException("Camera is disabled")

        # Retrieve last image from image topic
        self._receiving_image_front = None
        while self._receiving_image_front is None:
            time.sleep(0.002)
        image = self._receiving_image_front
        self._receiving_image_front = None
        return image

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
        self._pan_tilt_srv(
            Int16(pan_position),
            Int8(pan_speed),
            Int16(pan_blockid),
            Int16(0),
            Int8(0),
            Int16(0),
        )

    def read_phone_pan(self) -> int:
        """Get the current pan of the phone. Range: 0-100
        https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotpan
        """
        return self._pan

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
        self._pan_tilt_srv(
            Int16(0),
            Int8(0),
            Int16(0),
            Int16(tilt_position),
            Int8(tilt_speed),
            Int16(tilt_blockid),
        )

    def read_phone_tilt(self) -> int:
        """Get the current tilt of the phone. Range: 26-109
        https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robottilt
        """
        return self._tilt

    def read_accel(self) -> Acceleration:
        """Get the acceleration of the robot
        https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotaccel
        """
        return self._accel

    def read_orientation(self) -> Orientation:
        """Get the orientation of the robot
        https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotorientation
        """
        return self._orient

    def read_wheels(self) -> WheelPosition:
        """Get the wheel orientation and speed of the robot
        https://github.com/mintforpeople/robobo-programming/wiki/Robobo-Topics#robotwheels
        """
        return self._wheelpos

    def sleep(self, seconds: int) -> None:
        rospy.sleep(seconds)

    def _irs_callback(self, ros_data: IRs) -> None:
        self._irs_values = [
            ros_data.BackR.range,
            ros_data.BackC.range,
            ros_data.BackL.range,
            ros_data.FrontRR.range,
            ros_data.FrontR.range,
            ros_data.FrontC.range,
            ros_data.FrontL.range,
            ros_data.FrontLL.range,
        ]

    def _camera_callback_front(self, ros_data: CompressedImage):
        if self._receiving_image_front is None:
            #### direct conversion to CV2 ####
            np_arr = numpy.fromstring(ros_data.data, numpy.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # save result in memory (FLIP NEEDED [only front camera?])
            self._receiving_image_front = cv2.flip(image_np, 1)

    def _pan_callback(self, ros_data: Int16) -> None:
        self._pan = int(ros_data)

    def _tilt_callback(self, ros_data: Int16) -> None:
        self._tilt = int(ros_data)

    def _accel_callback(self, ros_data: Accel) -> None:
        self._accel = Acceleration(
            x=float(ros_data.linear.x),
            y=float(ros_data.linear.y),
            z=float(ros_data.linear.z),
        )

    def _orient_callback(self, ros_data: Quaternion) -> None:
        self._accel = Orientation(
            x=float(ros_data.x),
            y=float(ros_data.y),
            z=float(ros_data.z),
            w=float(ros_data.w),
        )

    def _wheelpos_callback(self, ros_data: Wheels) -> None:
        self._wheelpos = WheelPosition(
            wheel_pos_r=int(ros_data.wheelPosR),
            wheel_pos_l=int(ros_data.wheelPosL),
            wheel_speed_r=int(ros_data.wheelSpeedR),
            wheel_speed_l=int(ros_data.wheelSpeedL),
        )

    def spin(self):
        rospy.spin()
