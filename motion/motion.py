import sys
import json
import math
import time
sys.path.append('lowlevel')

sys.path.append('model')
sys.path.append('model/utils')
from KondoMVModel import KondoMVModel

sys.path.append('motion/moves')
from moves.head import Head
from moves.walk import Walk

from move_scheduler import MoveScheduler

class Motion:
    def __init__(self, unix=False, controller=True, imu=True):
        self.unix = unix
        
        # RCB4 controller init
        if controller:
            from controller_manager import ControllerManager
            self.cm = ControllerManager()
        else:
            self.cm = None
            print('no controller mode')

        # imu init
        if imu:
            from imu import IMU
            self.imu = IMU()
        else:
            self.imu = None
            print('no imu mode')


    '''def apply(self, action):
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
        return 0'''

if __name__ == "__main__":
    m = Motion(True, False, False)
    head = Head()

    model = KondoMVModel()
    walk = Walk(True, model)
    ms = MoveScheduler(model)
    walk.enabled = True
    head.enabled = True
    #walk.update(0.05, 0.0, 0, 0, 2)
    ms.start_move(walk)
    ms.start_move(head)
    while len(walk.frames_to_process) > 0:
        if len(walk.frames_to_process) == 0:
            ms.stop_move(walk)
        ms.tick()
        print(walk.frames_to_process)
        if m.cm is not None:
            m.cm.set_servos(ms.servos)
        

