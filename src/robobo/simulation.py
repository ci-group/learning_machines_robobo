from __future__ import unicode_literals, print_function, absolute_import, division, generators, nested_scopes
from robobo.base import Robobo
import time
import vrep
import cv2
import numpy as np

class VREPCommunicationError(Exception):
    pass

def _vrep_unwrap(result):
    if type(result) is tuple:
        ret_code = result[0]
        result = result[1:]
    else:
        ret_code = result
        result = None

    if ret_code < 0:
        raise VREPCommunicationError()
    return result

class SimulationRobobo(Robobo):
    def __init__(self, number=""):
        self._clientID = None
        self._value_number = number
    
    def connect(self, address='127.0.0.1', port=19999):
        vrep.simxFinish(-1)  # just in case, close all opened connections
        self._clientID = vrep.simxStart(address, port, True, True, 5000, 5)  # Connect to V-REP
        if self._clientID >= 0: #  and clientID_0 != -1:
            print('Connected to remote API server: client id {}'.format(self._clientID))
        else:
            raise VREPCommunicationError('Failed connecting to remote API server')

        self._RightMotor = self._vrep_get_object_handle('Right_Motor{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._LeftMotor = self._vrep_get_object_handle('Left_Motor{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._Robobo = self._vrep_get_object_handle('Robobo{}'.format(self._value_number), vrep.simx_opmode_blocking)

        self._IrBackC = self._vrep_get_object_handle('Ir_Back_C{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._IrFrontC = self._vrep_get_object_handle('Ir_Front_C{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._IrFrontLL = self._vrep_get_object_handle('Ir_Front_LL{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._IrFrontRR = self._vrep_get_object_handle('Ir_Front_RR{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._IrBackL = self._vrep_get_object_handle('Ir_Back_L{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._IrBackLFloor = self._vrep_get_object_handle('Ir_Back_L_Floor{}'.format(self._value_number),
                                                              vrep.simx_opmode_blocking)
        self._IrBackR = self._vrep_get_object_handle('Ir_Back_R{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._IrBackRFloor = self._vrep_get_object_handle('Ir_Back_R_Floor{}'.format(self._value_number),
                                                              vrep.simx_opmode_blocking)
        self._IrFrontL = self._vrep_get_object_handle('Ir_Front_L{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._IrFrontLFloor = self._vrep_get_object_handle('Ir_Front_L_Floor{}'.format(self._value_number),
                                                               vrep.simx_opmode_blocking)
        self._IrFrontR = self._vrep_get_object_handle('Ir_Front_R{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._IrFrontRFloor = self._vrep_get_object_handle('Ir_Front_R_Floor{}'.format(self._value_number),
                                                               vrep.simx_opmode_blocking)
        self._TiltMotor = self._vrep_get_object_handle('Tilt_Motor{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._PanMotor = self._vrep_get_object_handle('Pan_Motor{}'.format(self._value_number), vrep.simx_opmode_blocking)
        self._FrontalCamera = self._vrep_get_object_handle('Frontal_Camera{}'.format(self._value_number), vrep.simx_opmode_blocking)

        detectionStateIrFrontC, detectedPointIrFrontC, detectedObjectHandleIrFrontC, \
        detectedSurfaceNormalVectorIrFrontC = self._vrep_read_proximity_sensor(self._IrFrontC)
        
        detectionStateIrBackC, detectedPointIrIrBackC, detectedObjectHandleIrBackC, \
        detectedSurfaceNormalVectorIrBackC = self._vrep_read_proximity_sensor(self._IrBackC)
        
        detectionStateIrFrontLL, detectedPointIrFrontLL, detectedObjectHandleIrFrontLL, \
        detectedSurfaceNormalVectorIrFrontLL = self._vrep_read_proximity_sensor(self._IrFrontLL)
        
        detectionStateIrFrontRR, detectedPointIrFrontRR, detectedObjectHandleIrFrontRR, \
        detectedSurfaceNormalVectorIrFrontRR = self._vrep_read_proximity_sensor(self._IrFrontRR)
        
        detectionStateIrBackL, detectedPointIrBackL, detectedObjectHandleIrBackL, \
        detectedSurfaceNormalVectorIrBackL = self._vrep_read_proximity_sensor(self._IrBackL)
        
        detectionStateIrBackLFloor, detectedPointIrBackLFloor, detectedObjectHandleIrBackLFloor, \
        detectedSurfaceNormalVectorIrBackLFloor = self._vrep_read_proximity_sensor(self._IrBackLFloor)
        
        detectionStateIrBackR, detectedPointIrBackR, detectedObjectHandleIrBackR, \
        detectedSurfaceNormalVectorIrBackR = self._vrep_read_proximity_sensor(self._IrBackR)
        
        detectionStateIrBackRFloor, detectedPointIrBackRFloor, detectedObjectHandleIrBackRFloor, \
        detectedSurfaceNormalVectorIrBackRFloor = self._vrep_read_proximity_sensor(self._IrBackRFloor)
        
        detectionStateIrFrontR, detectedPointIrFrontR, detectedObjectHandleIrFrontR, \
        detectedSurfaceNormalVectorIrFrontR = self._vrep_read_proximity_sensor(self._IrFrontR)
        
        detectionStateIrFrontRFloor, detectedPointIrFrontRFloor, detectedObjectHandleIrFrontRFloor, \
        detectedSurfaceNormalVectorIrFrontRFloor = self._vrep_read_proximity_sensor(self._IrFrontRFloor)
        
        detectionStateIrFrontL, detectedPointIrFrontL, detectedObjectHandleIrFrontL, \
        detectedSurfaceNormalVectorIrFrontL = self._vrep_read_proximity_sensor(self._IrFrontL)
        
        detectionStateIrFrontLFloor, detectedPointIrFrontLFloor, detectedObjectHandleIrFrontLFloor, \
        detectedSurfaceNormalVectorIrFrontLFloor = self._vrep_read_proximity_sensor(self._IrFrontLFloor)

        resolution, image = self._vrep_get_vision_sensor_image(self._FrontalCamera, vrep.simx_opmode_streaming)

        self._vrep_get_ping_time()
        return self

    def _vrep_get_ping_time(self):
        return _vrep_unwrap(vrep.simxGetPingTime(self._clientID))[0]

    def _vrep_get_object_handle(self, name, opmode):
        return _vrep_unwrap(vrep.simxGetObjectHandle(self._clientID, name, opmode))[0]

    def _vrep_read_proximity_sensor(self, handle, opmode=vrep.simx_opmode_streaming):
        return _vrep_unwrap(vrep.simxReadProximitySensor(self._clientID, handle, opmode))

    def _vrep_get_vision_sensor_image(self, camera_handle, opmode=vrep.simx_opmode_buffer, a=0):
        return _vrep_unwrap(vrep.simxGetVisionSensorImage(self._clientID, camera_handle, a, opmode))

    def _vrep_set_joint_target_velocity(self, handle, speed, opmode):
        return _vrep_unwrap(vrep.simxSetJointTargetVelocity(self._clientID, handle, speed, opmode))

    def _vrep_set_joint_target_position(self, handle, position, opmode=vrep.simx_opmode_oneshot):
        return _vrep_unwrap(vrep.simxSetJointTargetPosition(self._clientID, handle, position, opmode))

    def spin(self):
        raise NotImplementedError("Not implemeted yet")

    def set_emotion(self, emotion):
        print("ROBOT EMOTION: {}".format(emotion))

    def move(self, left, right, millis=500):
        normalizer = 10.0
        left = left/normalizer
        right = right/normalizer

        self._vrep_set_joint_target_velocity(self._LeftMotor, left, vrep.simx_opmode_oneshot)
        self._vrep_set_joint_target_velocity(self._RightMotor, right, vrep.simx_opmode_oneshot)
        self._vrep_get_ping_time()

        duration = millis / 1000.0
        startTime = time.time()
        while time.time() - startTime < duration:
            rightMotorAngPos = _vrep_unwrap(vrep.simxGetJointPosition(self._clientID, self._RightMotor, vrep.simx_opmode_buffer))[0]
            leftMotorAngPos  = _vrep_unwrap(vrep.simxGetJointPosition(self._clientID, self._LeftMotor, vrep.simx_opmode_buffer))[0]
            RoboAbsPos       = _vrep_unwrap(vrep.simxGetObjectPosition(self._clientID, self._Robobo, -1, vrep.simx_opmode_buffer))[0]
            time.sleep(0.005)
        
        # Stop to move the wheels motor. Angular velocity.
        stopRightVelocity = stopLeftVelocity = 0
        self._vrep_set_joint_target_velocity(self._LeftMotor, stopLeftVelocity,
                                                  vrep.simx_opmode_oneshot)
        self._vrep_set_joint_target_velocity(self._RightMotor, stopRightVelocity,
                                                  vrep.simx_opmode_oneshot)
        self._vrep_get_ping_time()

    def talk(self, message):
        print("ROBOT SAYS: {}".format(message))

    def set_led(self, selector, color):
        raise NotImplementedError("Not implemeted yet")
    
    def read_irs(self):
        """
        returns sensor readings: [backR, backC, backL, frontRR, frontR, frontC, frontL, frontLL]
        """      
        detectionStateIrFrontC, detectedPointIrFrontC, detectedObjectHandleIrFrontC, \
        detectedSurfaceNormalVectorIrFrontC = self._vrep_read_proximity_sensor(
            self._IrFrontC, vrep.simx_opmode_buffer)
        detectionStateIrBackC, detectedPointIrIrBackC, detectedObjectHandleIrBackC, \
        detectedSurfaceNormalVectorIrBackC = self._vrep_read_proximity_sensor(
            self._IrBackC, vrep.simx_opmode_buffer)
        detectionStateIrFrontLL, detectedPointIrFrontLL, detectedObjectHandleIrFrontLL, \
        detectedSurfaceNormalVectorIrFrontLL = self._vrep_read_proximity_sensor(
            self._IrFrontLL, vrep.simx_opmode_buffer)
        detectionStateIrFrontRR, detectedPointIrFrontRR, detectedObjectHandleIrFrontRR, \
        detectedSurfaceNormalVectorIrFrontRR = self._vrep_read_proximity_sensor(
            self._IrFrontRR, vrep.simx_opmode_buffer)
        detectionStateIrBackL, detectedPointIrBackL, detectedObjectHandleIrBackL, \
        detectedSurfaceNormalVectorIrBackL = self._vrep_read_proximity_sensor(
            self._IrBackL, vrep.simx_opmode_buffer)

        detectionStateIrBackR, detectedPointIrBackR, detectedObjectHandleIrBackR, \
        detectedSurfaceNormalVectorIrBackR = self._vrep_read_proximity_sensor(
            self._IrBackR, vrep.simx_opmode_buffer)

        detectionStateIrFrontR, detectedPointIrFrontR, detectedObjectHandleIrFrontR, \
        detectedSurfaceNormalVectorIrFrontR = self._vrep_read_proximity_sensor(
            self._IrFrontR, vrep.simx_opmode_buffer)

        detectionStateIrFrontL, detectedPointIrFrontL, detectedObjectHandleIrFrontL, \
        detectedSurfaceNormalVectorIrFrontL = self._vrep_read_proximity_sensor(
            self._IrFrontL, vrep.simx_opmode_buffer)

        vect = [np.sqrt(detectedPointIrBackR[0]   ** 2 + detectedPointIrBackR[1]   ** 2 + detectedPointIrBackR[2]   ** 2)
                if detectionStateIrBackR   else False,
                np.sqrt(detectedPointIrIrBackC[0] ** 2 + detectedPointIrIrBackC[1] ** 2 + detectedPointIrIrBackC[2] ** 2)
                if detectionStateIrBackC   else False,
                np.sqrt(detectedPointIrBackL[0] ** 2   + detectedPointIrBackL[1]   ** 2 + detectedPointIrBackL[2]   ** 2)
                if detectionStateIrBackL   else False,
                np.sqrt(detectedPointIrFrontRR[0] ** 2 + detectedPointIrFrontRR[1] ** 2 + detectedPointIrFrontRR[2] ** 2)
                if detectionStateIrFrontRR else False,
                np.sqrt(detectedPointIrFrontR[0] ** 2  + detectedPointIrFrontR[1]  ** 2 + detectedPointIrFrontR[2]  ** 2)
                if detectionStateIrFrontR  else False,
                np.sqrt(detectedPointIrFrontC[0] ** 2  + detectedPointIrFrontC[1]  ** 2 + detectedPointIrFrontC[2]  ** 2)
                if detectionStateIrBackC   else False,
                np.sqrt(detectedPointIrFrontL[0] ** 2  + detectedPointIrFrontL[1]  ** 2 + detectedPointIrFrontL[2]  ** 2)
                if detectionStateIrFrontL  else False,
                np.sqrt(detectedPointIrFrontLL[0] ** 2 + detectedPointIrFrontLL[1] ** 2 + detectedPointIrFrontLL[2] ** 2)
                if detectionStateIrFrontLL else False]

        # old_min = 0
        # old_max = 0.20
        # new_min = 18000
        # new_max = 0
        # return [(((old_value - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min for old_value in vect]
        return vect

    def get_image_front(self):
        return self._get_image(self._FrontalCamera)

    def _get_image(self, camera):
        self._vrep_get_ping_time()

        # get image
        resolution, image = self._vrep_get_vision_sensor_image(camera)

        # reshape image
        image = image[::-1]
        im_cv2 = np.array(image, dtype=np.uint8)
        im_cv2.resize([resolution[0], resolution[1], 3])
        im_cv2 = cv2.flip(im_cv2, 1)

        return im_cv2

    def set_phone_pan(self, pan_position, pan_speed):
        """
        Command the robot to move the smartphone holder in the horizontal (pan) axis.

        Arguments

        pan_position: Angle to position the pan at.
        pan_speed: Movement speed for the pan mechanism.
        """
        # tilt_position = np.pi / 4.0
        self._vrep_set_joint_target_position(self._PanMotor, pan_position)
        self._vrep_get_ping_time()

    def set_phone_tilt(self, tilt_position, tilt_speed):
        """
        Command the robot to move the smartphone holder in the vertical (tilt) axis.

        Arguments

        tilt_position: Angle to position the tilt at.
        tilt_speed: Movement speed for the tilt mechanism.
        """
        # tilt_position = np.pi / 4.0
        self._vrep_set_joint_target_position(self._TiltMotor, tilt_position)
        self._vrep_get_ping_time()
