from lowlevel.kondo_controller import Rcb4BaseLib
from pyb import UART
import json
import utime

class Motion:
    def __init__(self):
        # RCB4 controller init
        self.kondo = Rcb4BaseLib()
        _uart = UART(1, 115200, parity=0)
        self.kondo.open(uart)
        
        # loading motion dictionary
        self.motions = json.open("motion/data_motion.json")
        
        # timer for motions
        self._motion_duration = None
        self._motion_start_time = None
        
        # head init
        self.head_motion_states = json.open("head_motion.json")
        self.head_enabled = false
        self.head_pitch = 0
        self.head_tilt = 0
        self.head_state = 0
        
        # False if controller is busy (playing motion).
    def _timer_permission_check(self):
        if self._motion_duration is not None and self._motion_start_time is not None:
        return _motion_start_time + _motion_duration <= utime.time()
        else:
            return False
            
    def _set_timer(self, duration):
        self._motion_duration = duration
        self._motion_start_time = utime.time()
        
    def do_motion(self, target_motion):
        self.current_motion = target_motion
        self._set_timer(self.current_motion['duration'])
        self.kondo.motionPlay(self.motions[self.current_motion['id'])
        
    def em(self):
        self.kondo.freeAllServos()
        
        
    # Init position of robot (home position)
    def init(self):
        self.current_motion = self.motions['home_position']
        self._set_timer(self.current_motion['duration'])
        self.kondo.motionPlay(self.motions[self.current_motion['id'])
        
    # walk position of robot (get_ready position)
    def walk(self):
        self.kondo.motionPlay(self.motions['get_ready']['id'])
        
    # discrete head motion that understands its position and does next step
    def move_head(self):
        if self.head_enabled:
            if self._timer_permission_check():
                self.head_state = (self.head_state + 1) % len(self.head_motion_states)
                self.head_pitch = self.head_motion_states[self.head_state]['pitch']
                self.head_tilt = self.head_motion_states[self.head_state]['tilt']
                self.kondo.setUserParameter(20, self.head_pitch)
                self.kondo.setUserParameter(19, self.head_tilt)
            
        
        

    def apply(self, action):
        if motion_timer is not None and motion_timer == 0.0:
            if action['step_forward'] != 0:
                pass
        
        return 0
