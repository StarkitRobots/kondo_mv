import sys
sys.path.append('../lowlevel')
from kondo_controller import Rcb4BaseLib
from pyb import UART
import json
import utime

class Motion:
    def __init__(self):
        # RCB4 controller init
        self.kondo = Rcb4BaseLib()
        _uart = UART(1, 115200, parity=0)
        self.kondo.open(_uart)
        
        # loading motion dictionary
        self.motions = json.load("/motion/kondo_motions.json")
        
        # timer for motions
        self._motion_duration = None
        self._motion_start_time = None
        
        # head init
        self.head_motion_states = json.load("/motion/head_motion.json")
        self.head_enabled = False
        self.head_pitch = 0
        self.head_tilt = 0
        self.head_state = 0
        
        # False if controller is busy (playing motion).
    def _timer_permission_check(self):
        if self._motion_duration is not None and self._motion_start_time is not None:
            return self._motion_start_time + self._motion_duration <= utime.time()
        else:
            return False
            
    def _set_timer(self, duration):
        self._motion_duration = duration
        self._motion_start_time = utime.time()
        
    def do_motion(self, target_motion, args=None):
        self.current_motion = target_motion
        self._set_timer(self.current_motion['duration'])
        if args is not None:
            if args['c1'] != 0:
                self.kondo.setUserCounter(1, args['c1'])
            if args['u1'] != 0:
                self.kondo.setUserParameter(1, args['u1'])

        self.kondo.motionPlay(self.current_motion['id'])


    # does not work now    
    def em(self):
        self.kondo.freeAllServos()
        
        
    # Init position of robot (home position)
    def init(self):
        if self._timer_permission_check():
            self.do_motion(self.motions['home_position'])
        
    # walk position of robot (get_ready position)
    def walk(self):
        if self._timer_permission_check():
            self.do_motion(self.motions['get_ready'])

    # discrete head motion that understands its position and does next step
    def move_head(self):
        if self.head_enabled:
            if self._timer_permission_check():
                self.head_state = (self.head_state + 1) % len(self.head_motion_states)
                self.head_pitch = self.head_motion_states[self.head_state]['pitch']
                self.head_tilt = self.head_motion_states[self.head_state]['tilt']
                self.kondo.setUserParameter(20, self.head_pitch)
                self.kondo.setUserParameter(19, self.head_tilt)
            else:
                pass
        else:
            pass
            
        
    def apply(self, action):

        if action['name'] == 'walk':
            self.do_motion(self.motions['Soccer_WalkFF'])    
        
        return 0
