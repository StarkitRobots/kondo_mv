import sys
import json
import math
import time
sys.path.append('../lowlevel')
from imu import IMU
from controller_manager import ControllerManager

class Motion:
    def __init__(self, unix=False, controller=True, imu=True):
        self.unix = unix
        
        # RCB4 controller init
        if controller:
            self.cm = ControllerManager()
        else:
            self.cm = None
            print('no controller mode')

        # imu init
        if imu:
            self.imu = IMU()
        else:
            self.imu = None
            print('no imu mode')


    def apply(self, action):
        if action['name'] == 'walk':
            return self._walk_control_motions(action['args'])
        elif action['name'] == 'turn':
            return self._turn_control(action['args'])
        elif action['name'] == 'kick':
            return self._kick_control(action['args'])
        elif action['name'] == 'lateral_step':
            return  self._lateral_control(action['args'])
        elif action['name'] == 'take_around_right':
            return self.do_motion(self.motions['Soccer_Take_Around_Right'] , {'c1': 1, 'u1': 0})
        return 0

if __name__ == "__main__":
    m = Motion(True, False, False)
    print(m.apply({'name': 'turn', 'args':(0.2)}))
    print(m.apply({'name': 'walk', 'args':(0.3, 0.1)}))

