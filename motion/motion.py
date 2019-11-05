import sys
sys.path.append('../lowlevel')
from kondo_controller import Rcb4BaseLib
from machine import I2C
from bno055 import BNO055, AXIS_P7
from pyb import UART
import json, math
import time
sys.path.append('motion')
from motion import *


###########################################################################################
# utils
###########################################################################################

def degrees_to_head(degrees):
    return int(int(degrees) * 8000. / 270.)

###########################################################################################

class Motion:
    def __init__(self):
        # RCB4 controller init
        self.kondo = Rcb4BaseLib()
        _uart = UART(1, 115200, parity=0, timeout = 1000)
        self.kondo.open(_uart)
        print(self.kondo.checkAcknowledge())

        # imu init
        i2c = I2C(2)
        self.imu = BNO055(i2c)

        # loading motion dictionary

        #self.motions = json.load("/motion/data_motion.json")

        self.motions = {
            "Soccer_WALK_FF" : {
                "id"        : 8,
                "time"      : (lambda c1 : 230 * c1 + 440),
                "shift_x"   : (lambda c1, u1 : 4.3 / math.sin(u1 * 0.0140625 * math.pi / 180.) *
                                math.sin(u1 * 0.028125 * c1 * math.pi / 180.) / 100.),
                "shift_y"   : (lambda c1, u1 : -4.3 / math.sin(u1 * 0.0140625 * math.pi / 180.) +
                                4.3 / math.sin(u1 * 0.0140625 * math.pi / 180.) *
                                math.cos(u1 * 0.028125 * c1 * math.pi / 180.) / 100.),
                "shift_turn": (lambda u1 : u1 * 0.028125)
                },

            "Soccer_Turn" : {
                "id"        : 15,
                "time"      : (lambda c1 : 220 * c1 + 80),
                "shift_x"   : 0,
                "shift_y"   : 0,
                "shift_turn": (lambda c1, u1 : u1 * 0.12 * c1)
                },

            "Soccer_Side_Step_Left" : {
                "id"        : 11,
                "time"      : (lambda c1 : 180 * c1 + 360),
                "shift_x"   : 0,
                "shift_y"   : (lambda c1: -11. * c1 / 100.),
                "shift_turn": 0
                },

            "Soccer_Side_Step_Right" : {
                "id"        : 10,
                "time"      : (lambda c1 : 180 * c1 + 360),
                "shift_x"   : 0,
                "shift_y"   : (lambda c1: 11. * c1 / 100.),
                "shift_turn": 0
                },

            "Soccer_Small_Step_Left" : {
                "id"        : 13,
                "time"      : (lambda c1 : 1250 * c1 + 150),
                "shift_x"   : 0,
                "shift_y"   : (lambda c1: -3.3 * c1 / 100.),
                "shift_turn": 0
                },

            "Soccer_Small_Step_Right" : {
                "id"        : 12,
                "time"      : (lambda c1 : 1250 * c1 + 150),
                "shift_x"   : 0,
                "shift_y"   : (lambda c1: 3.3 * c1 / 100.),
                "shift_turn": 0
                },

            "Soccer_Take_Around_Left" : {
                "id"        : 17,
                "time"      : (lambda c1 : 1250 * c1 + 150),
                "shift_x"   : (lambda c1 : 1.65 / math.sin(6.3 * math.pi / 180) *
                                (1 - math.cos(12.6 * c1 * math.pi / 180)) / 100.),
                "shift_y"   : (lambda c1 : -1.65 / math.sin(6.3 * math.pi / 180) *
                                math.sin(12.6 * c1 * math.pi / 180) / 100.),
                "shift_turn": (lambda c1 : -12.6 * c1)
                },

            "Soccer_Take_Around_Right" : {
                "id"        : 16,
                "time"      : (lambda c1 : 1250 * c1 + 150),
                "shift_x"   : (lambda c1 : 1.65 / math.sin(6.3 * math.pi / 180) *
                                (1 - math.cos(12.6 * c1 * math.pi / 180)) / 100.),
                "shift_y"   : (lambda c1: 1.65 / math.sin(6.3 * math.pi / 180) *
                                math.sin(12.6 * c1 * math.pi / 180) / 100.),
                "shift_turn": (lambda c1 : 12.6 * c1)
                },

            "Soccer_Kick Forward_Left_leg " : {
                "id"        : 19,
                "time"      : 3000,
                "shift_x"   : None,
                "shift_y"   : None,
                "shift_turn": None
                },

            "Soccer_Kick Forward_Right_leg " : {
                "id"        : 18,
                "time"      : 3000,
                "shift_x"   : None,
                "shift_y"   : None,
                "shift_turn": None
                },

            "Soccer_HomePosition" : {
                "id"        : 1,
                "time"      : 1000
                },

            "Soccer_Get_Ready" : {
                "id"        : 2,
                "time"      : 1000
                },

            "Free" : {
                "id"        : 4,
                "time"      : 1000
                }
            }


        # timer for motions
        self._motion_duration = 0.0
        self._motion_start_time = 0.0

        # head init
        with open("motion/head_motion.json", "r") as f:
            self.head_motion_states = json.loads(f.read())
        self.head_state_num = len(self.head_motion_states)
        self.head_enabled = True
        self.head_yaw = 0
        self.head_pitch = 0
        self.head_state = 0

        # walk threshold
        self.walk_threshold = 1.0
        self.max_blind_distance = 0.4
        self.step_len = 0.086

        # acceptable error in degrees
        self.angle_error_treshold = 5


###########################################################################################
# timer methods
###########################################################################################

    # False if controller is busy (playing motion).
    def _timer_permission_check(self):
        return self._motion_start_time + self._motion_duration <= time.ticks()

    def _get_timer_duration(self, motion, args):
        return motion['time'](args['c1'])

    def _set_timer(self, duration):
        self._motion_duration = duration
        self._motion_start_time = time.ticks()
        time.sleep(int(duration))


###########################################################################################
# several direct motion methods
###########################################################################################

    # basic method of any motion appliance
    def do_motion(self, target_motion, args=None):
        if self._timer_permission_check():
            self.current_motion = target_motion
            if args is not None:
                if args['c1'] != 0:
                    self.kondo.setUserCounter(1, args['c1'])
                if args['u1'] != 0:
                    self.kondo.setUserParameter(1, args['u1'])

            self.kondo.motionPlay(self.current_motion['id'])
            self._set_timer(self._get_timer_duration(self.current_motion, args))
        return target_motion['shift_x'], target_motion['shift_y'],


    # Free all servos (Almost similar to rhoban 'em' command)
    def em(self):
        self.kondo.do_motion(self.motions['Free'])


    # home position of robot (rhoban init position)
    def init(self):
        self.do_motion(self.motions['Soccer_HomePosition'])

    # ready position of robot (rhoban walk position)
    def ready(self):
        self.do_motion(self.motions['Soccer_Get_Ready'])

    def kick(self, left=True):
        self._kick_control({'left' : left})

    # discrete head motion that understands its position and does next step
    def move_head(self):
        if self.head_enabled:
            if self._timer_permission_check():
                self.head_state = (self.head_state + 1) % self.head_state_num
                self.head_pitch = int(self.head_motion_states[str(self.head_state)]['pitch'])
                self.head_yaw = int(self.head_motion_states[str(self.head_state)]['yaw'])
                self.kondo.setUserParameter(20, degrees_to_head(self.head_pitch))
                self.kondo.setUserParameter(19, degrees_to_head(self.head_yaw))
                self._set_timer(500)
                #print("pitch " + str(self.kondo.getSinglePos(1)[1] + " yaw " + self.kondo.getSinglePos(1)[1])
                return self.head_pitch, self.head_yaw
            else:
                pass
        else:
            pass


###########################################################################################
# hidden methods of motion control according to their arguments
###########################################################################################

    # hardcode piece of Kefir code (don't do it like that next time)
    def _get_turn_params(self, target, func):
        uo = -150
        co = 1

        err = 360

        if (abs (target) > 360):
            print ("idi naher")

        neg = False

        if target < 0:
            neg = True
            target *= -1

        for u in range(-150, 150, 10):
            for c in range(1, target // 15 + 1):
                curr = func(c, u)
                curr_err = abs (target - curr)

                if curr_err < err:
                    err = curr_err
                    uo = u
                    co = c

        if (neg == True):
            uo *= -1
        return co, uo

    def _turn_control(self, turn_args):
        rotation_angle = turn_args * 180. / math.pi
        #rotation_angle = math.atan(x / y) / math.pi * 180.
        motion = self.motions['Soccer_Turn']
        c1, u1 = self._get_turn_params(rotation_angle, motion['shift_turn'])

        if rotation_angle > self.angle_error_treshold:
            self.do_motion(motion, {'c1': c1, 'u1': u1})
            yaw, roll, pitch = self.imu.euler()
            return {"shift_x": motion['shift_x'], "shift_y": motion['shift_y'], "yaw": yaw}
        else:
            yaw, roll, pitch = self.imu.euler()
            return {"shift_x": 0, "shift_y": 0, "yaw": yaw}

    def _walk_control(self, walk_args):
        distance = walk_args[0]
        rotation_angle = walk_args[1]
        if rotation_angle > self.angle_error_treshold:
            self._turn_control(rotation_angle)
        else:
            motion = self.motions['Soccer_WALK_FF']
            #distance = math.sqrt(x*x + y*y)
            if distance < self.max_blind_distance:
                step_num = distance // self.step_len
            else:
                step_num = self.max_blind_distance // self.step_len
            self.do_motion(motion, {'c1': step_num, 'u1': 1})
            return {"shift_x": motion['shift_x'](step_num, 1),
                "shift_y": motion['shift_y'](step_num, 1), "yaw": self.imu}

    def _kick_control(self, kick_args):
        if kick_args['left']:
            self.do_motion(self.motions['Soccer_Kick Forward_Left_leg'])
        else:
            self.do_motion(self.motions['Soccer_Kick Forward_Right_leg'])

    def _lateral_control(self, lateral_args):
        pass

###########################################################################################
#
###########################################################################################

    def apply(self, action):
        if self._timer_permission_check():
            if action['name'] == 'walk':
                self._walk_control(action['args'])
            elif action['name'] == 'turn':
                self._turn_control(action['args'])
            elif action['name'] == 'kick':
                self._kick_control(action['args'])
            elif action['name'] == 'lateral_step':
                self._lateral_control(action['args'])
            elif action['name'] == 'take_around':
                pass
        return 0
