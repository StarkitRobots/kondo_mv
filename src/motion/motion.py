import json
import math
import time

from .moves import Head, Walk, Kick
from .move_scheduler import MoveScheduler
from .controller_manager import ControllerManager
from ..odometry import Odometry

class Motion:
    def __init__(self, model, unix=False, controller=None, imu=None):
        self.unix = unix
        
        # RCB4 controller init
        if controller is not None:
            self.controller = ControllerManager(controller)
        else:
            self.controller = None
            print('No controller mode')

        # imu init
        self.imu = imu
        if self.imu is None:
            print('No imu mode')
        self.model = model
        self.odometry = Odometry(self.imu)
        self.move_scheduler = MoveScheduler(self.model)
        self.walk = Walk(model=self.model)
        self.head = Head()
        self.kick = Kick()
        self.head.enabled = True
    
    def apply(self, action):
        if action['name'] == 'head':
            self.walk.enabled = False
            self.head.enabled = True
        if not self.move_scheduler.has_active_move('head') and self.head.enabled:
            self.move_scheduler.start_move(self.head)
        elif action['name'] == 'walk':
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
                self.odometry.shift_x = step_params[0]
                self.odometry.shift_y = step_params[1]
                
        elif action['name'] == 'kick':
            if self.move_scheduler.has_active_move('walk'):
                self.move_scheduler.stop_move(self.walk)
            self.move_scheduler.start_move(self.kick)
            self.kick.side = action['args']
        self.move_scheduler.tick()
        if not self.move_scheduler.has_active_move('head') and self.head.enabled:
            self.move_scheduler.start_move(self.head)
        self.controller.set_servos(self.move_scheduler.servos)
        return 0

if __name__ == "__main__":
    from ..model import KondoMV
    model = KondoMV()
    m = Motion(model, True, False, False)
    m.apply({'name': 'kick', 'args': -1})
    