import json
import math
import time

from .moves import Head, Walk, Kick
from .move_scheduler import MoveScheduler

class Motion:
    def __init__(self, model, unix=False, controller=None, imu=None, odometry=None):
        self.unix = unix
        
        # RCB4 controller init
        if controller:
            from controller_manager import ControllerManager
            self.controller_manager = ControllerManager()
        else:
            self.cm = None
            print('No controller mode')

        # imu init
        self.imu = imu
        if self.imu is None:
            print('No imu mode')
        self.model = model
        self.move_scheduler = MoveScheduler(self.model)
        self.walk = Walk(True, self.model)
        self.head = Head()
        self.kick = Kick()
        self.head.enabled = True
        self.odometry = odometry
        if self.odometry is None:
            print('No odometry mode. Kondo motions are not availible')
    
    def apply(self, action):
        if action['name'] == 'walk':
            if not self.move_scheduler.has_active_move('walk'):
                self.walk.enabled = True
                self.move_scheduler.start_move(self.walk)
                self.walk.cycle = 0
            else:
                strategy_bus = action['args']
                step_params = strategy_bus[0]
                cycles_num = len(strategy_bus)
                self.walk.update(walk_step=step_params[0], 
                                    lateral_step=step_params[1], 
                                    walk_turn=step_params[2], 
                                    cycle=self.walk.cycle, 
                                    cycles_num=cycles_num)
                self.walk.cycle += 1
        elif action['name'] == 'kick':
            if self.move_scheduler.has_active_move('walk'):
                self.move_scheduler.stop_move(self.walk)
            self.move_scheduler.start_move(self.kick)
            self.kick.side = action['args']
        self.move_scheduler.tick()
        motion = self.move_scheduler.get_kondo_motion()
        if self.move_scheduler._is_motion(motion):
            motion_to_apply = self.odometry.motions[motion['name']]
            self.controller_manager.do_motion(motion_to_apply, motion['args'])
        else:
            if not self.head.enabled:
                self.move_scheduler.start_move(self.head)
            self.controller_manager.servos(self.move_scheduler.servos)
        return 0

if __name__ == "__main__":
    from ..model import KondoMV
    model = KondoMV()
    m = Motion(model, True, False, False)
    m.apply({'name': 'kick', 'args': -1})
        
        

