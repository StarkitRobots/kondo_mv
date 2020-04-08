import time
from . import vrep
from . import vrep_consts

class SimulationController:
    def __init__(self, servos, *args):
        self.client_id = -1
        try:
            self.client_id = args[0]
        except IndexError:
            raise Exception('Failed connecting to remote API server. \n \
                No client_id provided to SimulationController')
        if len(args) == 2:
            self.use_physics = args[1]
        else:
            self.use_physics = True

        self.servos = servos
        self.simulation_joints = {}
        # Connect to V-REP
        if self.check_acknowledge:
            print ('Connected to remote API server')
        else:
            raise Exception('Failed connecting to remote API server')
        
        # Collect Joint Handles and trims from model
        for servo in self.servos:
            _, simulation_joint = vrep.simxGetObjectHandle(self.client_id, 
                                            self.servos[servo]['simulation_joint_name'], 
                                            vrep_consts.simx_opmode_blocking)
            self.simulation_joints[servo] = simulation_joint

        if self.use_physics:
            vrep.simxSynchronous(self.client_id, True)

    def check_acknowledge(self):
        return self.client_id != -1

    def set_servos(self, servo_data):
        for servo in servo_data:
            if self.use_physics: 
                vrep.simxSetJointTargetPosition(self.client_id, 
                                                self.simulation_joints[servo], 
                                                servo_data[servo] * self.servos[servo]['inverse'], 
                                                vrep_consts.simx_opmode_oneshot)
            else: 
                vrep.simxSetJointPosition(self.client_id, 
                                            self.simulation_joints[servo], 
                                            servo_data[servo] * self.servos[servo]['inverse'], 
                                            vrep_consts.simx_opmode_oneshot)

    def falling_test(self):
        pass
    