import vrep
from robobo import SimulationRobobo


class SimulationRoboboPrey(SimulationRobobo):
    def __init__(self, number="#0"):
        super(SimulationRoboboPrey, self).__init__(number)

    def initialize_handles(self):
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
        # read a first value in streaming mode
        self._vrep_read_proximity_sensor_ignore_error(self._IrFrontC)
        self._vrep_read_proximity_sensor_ignore_error(self._IrBackC)
        self._vrep_read_proximity_sensor_ignore_error(self._IrFrontLL)
        self._vrep_read_proximity_sensor_ignore_error(self._IrFrontRR)
        self._vrep_read_proximity_sensor_ignore_error(self._IrBackL)
        self._vrep_read_proximity_sensor_ignore_error(self._IrBackLFloor)
        self._vrep_read_proximity_sensor_ignore_error(self._IrBackR)
        self._vrep_read_proximity_sensor_ignore_error(self._IrBackRFloor)
        self._vrep_read_proximity_sensor_ignore_error(self._IrFrontR)
        self._vrep_read_proximity_sensor_ignore_error(self._IrFrontRFloor)
        self._vrep_read_proximity_sensor_ignore_error(self._IrFrontL)
        self._vrep_read_proximity_sensor_ignore_error(self._IrFrontLFloor)

        self.wait_for_ping()
