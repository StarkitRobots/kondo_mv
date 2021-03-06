import sys
sys.path.append('../lowlevel')
import json, math
import time
sys.path.append('motion')
from motion import *


CHANNEL_WAIT_TIME = 10

###########################################################################################
# utils
###########################################################################################

def degrees_to_head(degrees):
    return int(int(degrees) * 8000. / 270.)

###########################################################################################

class Motion:
    def __init__(self, unix=False, controller=True, imu=True):
        self.unix = unix

        # RCB4 controller init
        if controller:
            from kondo_controller import Rcb4BaseLib
            from pyb import UART

            self.kondo = Rcb4BaseLib()
            _uart = UART(1, 115200, parity=0, timeout = 1000)
            self.kondo.open(_uart)
            print(self.kondo.checkAcknowledge())
        else:
            self.kondo = None
            print('no controller mode')

        # imu init
        if imu:
            from machine import I2C
            from bno055 import BNO055, AXIS_P7

            i2c = I2C(2)
            self.imu = BNO055(i2c)
        else:
            self.imu = None
            print('no imu mode')

        # loading motion dictionary

        #self.motions = json.load("/motion/data_motion.json")
        self.motion_to_apply = None
        self.motions = {
            "Soccer_WALK_FF" : {
                "id"        : 8,
                "time"      : (lambda c1, u1 : 230 * c1 + 440),
                "shift_x"   : (lambda c1, u1 : 4.3 / math.sin(u1 * 0.0140625 * math.pi / 180.) *
                                math.sin(u1 * 0.028125 * c1 * math.pi / 180.) / 100.),
                "shift_y"   : (lambda c1, u1 : -4.3 / math.sin(u1 * 0.0140625 * math.pi / 180.) / 100 +
                                4.3 / math.sin(u1 * 0.0140625 * math.pi / 180.) *
                                math.cos(u1 * 0.028125 * c1 * math.pi / 180.) / 100.),
                "shift_turn": (lambda c1, u1 : u1 * 0.028125)
                },

            "Soccer_Turn" : {
                "id"        : 15,
                "time"      : (lambda c1, u1 : 220 * c1 + 80),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: 0),
                "shift_turn": (lambda c1, u1 : -1* u1 * 0.12 * c1)
                },

            "Soccer_Side_Step_Left" : {
                "id"        : 11,
                "time"      : (lambda c1, u1 : 180 * c1 + 360),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: 11. * c1 / 100.),
                "shift_turn": (lambda c1, u1: 0)
                },

            "Soccer_Side_Step_Right" : {
                "id"        : 10,
                "time"      : (lambda c1, u1 : 180 * c1 + 360),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: -11. * c1 / 100.),
                "shift_turn": (lambda c1, u1: 0)
                },

            "Soccer_Small_Step_Left" : {
                "id"        : 13,
                "time"      : (lambda c1, u1 : 1250 * c1 + 150),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: 3.3 * c1 / 100.),
                "shift_turn": (lambda c1, u1: 0)
                },

            "Soccer_Small_Step_Right" : {
                "id"        : 12,
                "time"      : (lambda c1, u1 : 1250 * c1 + 150),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: -3.3 * c1 / 100.),
                "shift_turn": (lambda c1, u1: 0)
                },

            "Soccer_Take_Around_Left" : {
                "id"        : 17,
                "time"      : (lambda c1, u1 : 1250 * c1 + 150),
                "shift_x"   : (lambda c1, u1 : 1.65 / math.sin(6.3 * math.pi / 180) *
                                (1 - math.cos(12.6 * c1 * math.pi / 180)) / 100.),
                "shift_y"   : (lambda c1, u1 : 1.65 / math.sin(6.3 * math.pi / 180) *
                                math.sin(12.6 * c1 * math.pi / 180) / 100.),
                "shift_turn": (lambda c1, u1 : 12.6 * c1)
                },

            "Soccer_Take_Around_Right" : {
                "id"        : 16,
                "time"      : (lambda c1, u1 : 1250 * c1 + 150),
                "shift_x"   : (lambda c1, u1 : 1.65 / math.sin(6.3 * math.pi / 180) *
                                (1 - math.cos(12.6 * c1 * math.pi / 180)) / 100.),
                "shift_y"   : (lambda c1, u1: -1.65 / math.sin(6.3 * math.pi / 180) *
                                math.sin(12.6 * c1 * math.pi / 180) / 100.),
                "shift_turn": (lambda c1, u1 : -12.6 * c1)
                },

            "Soccer_Kick_Forward_Left_leg" : {
                "id"        : 19,
                "time"      : (lambda c1, u1: 1000),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: 0),
                "shift_turn": (lambda c1, u1: 0)
                },

            "Soccer_Kick_Forward_Right_leg" : {
                "id"        : 18,
                "time"      : (lambda c1, u1: 1000),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: 0),
                "shift_turn": (lambda c1, u1: 0)
                },

            "Soccer_HomePosition" : {
                "id"        : 1,
                "time"      : (lambda c1, u1: 1000),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: 0),
                "shift_turn": (lambda c1, u1: 0)
                },

            "Soccer_Get_Ready" : {
                "id"        : 2,
                "time"      : (lambda c1, u1: 1000),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: 0),
                "shift_turn": (lambda c1, u1: 0)
                },

            "Free" : {
                "id"        : 4,
                "time"      : (lambda c1, u1: 1000),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: 0),
                "shift_turn": (lambda c1, u1: 0)
                },
            "Move_Head" : {
                "id"        : 112,
                "time"      : (lambda c1, u1: 600),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: 0),
                "shift_turn": (lambda c1, u1: 0)
                },
            "Empty" : {
                "id"        : 100,
                "time"      : (lambda c1, u1: 0),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: 0),
                "shift_turn": (lambda c1, u1: 0)
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
        self.head_pan = 0
        self.head_tilt = 0
        self.head_state = -1

        # get config params
        with open("motion/config.json", "r") as f:
            self.motion_config = json.loads(f.read())

        # walk threshold
        self.walk_threshold = self.motion_config['walk_threshold']
        self.max_blind_distance = self.motion_config['max_blind_distance']
        self.step_len = self.motion_config['step_len']

        # acceptable error in degrees
        self.angle_error_treshold = self.motion_config['angle_error_treshold'] * math.pi / 180.


    def get_imu_yaw(self):
        if self.imu is not None:
            try:
                yaw, pitch, roll = self.imu.euler()
            except OSError:
                time.sleep(CHANNEL_WAIT_TIME)
                yaw, pitch, roll = self.imu.euler()
        else:
            print('No imu. Unable to get yaw. Yaw is going to be zero')
            yaw = 0
        return -(yaw / 180. * math.pi)

    def get_odometry(self, motion, c1, u1, yaw_diff):
        return {'shift_x': motion['shift_x'](c1, u1), 'shift_y': motion['shift_y'](c1, u1), 'shift_yaw': yaw_diff}


###########################################################################################
# timer methods
###########################################################################################

    # False if controller is busy (playing motion).
    def _timer_permission_check(self):
        if self.unix:
            timer_check = self._motion_start_time + self._motion_duration <= time.time()
        else:
            timer_check = self._motion_start_time + self._motion_duration <= time.ticks()
        motion_finished = False
        while not (motion_finished and timer_check):
            try:
                if self.unix:
                    time.sleep(0.1)
                else:
                    time.sleep(100)
                if self.kondo is not None:
                    current_motion = self.kondo.getMotionPlayNum()
            except OSError:
                if self.unix:
                    time.sleep(CHANNEL_WAIT_TIME / 1000)
                else:
                    time.sleep(CHANNEL_WAIT_TIME)
                if self.kondo is not None:
                    current_motion = self.kondo.getMotionPlayNum()
            if self.kondo is not None:
                motion_finished = current_motion == 0 or current_motion == 1 or current_motion == 2
            else:
                motion_finished = True
        return True

    def _get_timer_duration(self, motion, args):
        return motion['time'](args['c1'], args['u1'])

    def _set_timer(self, duration):
        self._motion_duration = duration
        if self.unix:
            self._motion_start_time = time.time()
            time.sleep(int(duration / 1000))
        else:
            self._motion_start_time = time.ticks()
            time.sleep(int(duration))



###########################################################################################
# several direct motion methods
###########################################################################################

    # basic method of any motion appliance
    def do_motion(self, target_motion, args=None):
        c1 = 0
        u1 = 0
        if self._timer_permission_check():
            self.current_motion = target_motion
            yaw_before = self.get_imu_yaw()
            if self.kondo is not None:
                if args is not None:
                    if args['c1'] != 0:
                        c1 = args['c1']
                        self.kondo.setUserCounter(1, c1)
                    if args['u1'] != 0:
                        u1 = args['u1']
                        self.kondo.setUserParameter(1, u1)
                try:
                    self.kondo.motionPlay(self.current_motion['id'])
                except OSError:
                    time.sleep(CHANNEL_WAIT_TIME)
                    self.kondo.motionPlay(self.current_motion['id'])
                self._set_timer(self._get_timer_duration(self.current_motion, args))
                while not self._timer_permission_check():
                    time.sleep(100)
            else:
                print('No controller. Unable to perform a motion. Calculating odometry')
                if args['c1'] != 0:
                        c1 = args['c1']
                if args['u1'] != 0:
                    u1 = args['u1']
            yaw_after = self.get_imu_yaw()

        try:
            return self.get_odometry(self.current_motion, c1, u1, yaw_after - yaw_before)
        except KeyError:
            return self.get_odometry(self.motions['Empty'], 0, 0, 0)


    # Free all servos (Almost similar to rhoban 'em' command)
    def em(self):
        self.do_motion(self.motions['Free'])


    # home position of robot (rhoban init position)
    def init(self):
        self.do_motion(self.motions['Soccer_HomePosition'])

    # ready position of robot (rhoban walk position)
    def ready(self):
        self.do_motion(self.motions['Soccer_Get_Ready'])

    def kick(self, side=1):
        self._kick_control(side)

    # discrete head motion that understands its position and does next step
    def move_head(self):
        if self.head_enabled:
            if self._timer_permission_check():
                self._set_timer(150)
                self.head_state = (self.head_state + 1) % self.head_state_num
                self.head_pan = int(self.head_motion_states[str(self.head_state)]['yaw'])
                self.head_tilt = int(self.head_motion_states[str(self.head_state)]['pitch'])
                self.kondo.setUserParameter(20, degrees_to_head(self.head_tilt))
                self.kondo.setUserParameter(19, degrees_to_head(-self.head_pan))
                self._set_timer(150)
                #print("pitch " + str(self.kondo.getSinglePos(1)[1] + " yaw " + self.kondo.getSinglePos(1)[1])
                return self.head_pan * math.pi / 180., self.head_tilt * math.pi / 180.
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
        rotation_angle = turn_args / math.pi * 180.
        #rotation_angle = math.atan(x / y) / math.pi * 180.
        motion = self.motions['Soccer_Turn']
        c1, u1 = self._get_turn_params(rotation_angle, motion['shift_turn'])

        if abs(rotation_angle) > self.angle_error_treshold:
            return self.do_motion(motion, {'c1': c1, 'u1': u1})
        else:
            return self.get_odometry(self.motions['Empty'], 0, 0, 0)

    def _walk_control_motions(self, walk_args):
        distance = walk_args[0]
        rotation_angle = walk_args[1]
        if abs(rotation_angle) > self.angle_error_treshold:
            return self._turn_control(rotation_angle)
        else:
            motion = self.motions['Soccer_WALK_FF']
            #distance = math.sqrt(x*x + y*y)
            if distance < self.max_blind_distance:
                step_num = 1
            else:
                step_num = 3

            return self.do_motion(motion, {'c1': step_num, 'u1': 1})

    """def _walk_control_engine(self)
        for stop_point in range (len(destination)):
            x1, y1, z1 = motion.Dummy_HData[len(motion.Dummy_HData)-1]
            x1 = 1000* x1
            y1 = 1000* y1
            u1 = motion.euler_angle[0]
            x2,y2,u2 = destination[stop_point]
            x2 = 1000* x2
            y2 = 1000* y2
            step_Seq, walk_Direction = steps( x1 ,y1,u1,x2,y2,u2)
            for i in range (len(step_Seq)):
                motion.walk_Initial_Pose()
                stepLength, sideLength, rotation, cycleNumber = step_Seq[i]
                for cycle in range(cycleNumber):
                    rotation1 = rotation
                    if rotation == 0:
                        rotation1 = (walk_Direction - motion.euler_angle[0])*1
                        if rotation1 > 180 : rotation1 = rotation1 - 360
                        if rotation1 < -180 : rotation1 = rotation1 + 360
                        if rotation1 > 12 : rotation1 = 12
                        if rotation1 < -12 : rotation1 = -12

                    motion.walk_Cycle(stepLength, sideLength,rotation1,cycle,cycleNumber)
                motion.walk_Final_Pose()
            motion.turn_To_Course(u2)

    def placer(self, destination)
"""
    def _kick_control(self, kick_args):
        if kick_args == -1:
            return self.do_motion(self.motions['Soccer_Kick_Forward_Left_leg'], {'c1': 0, 'u1': 0})
        else:
            return self.do_motion(self.motions['Soccer_Kick_Forward_Right_leg'], {'c1': 0, 'u1': 0})

    def _lateral_control(self, lateral_args):
        step_num = int(lateral_args / 0.033)
        if step_num > 0:
            return self.do_motion(self.motions['Soccer_Small_Step_Left'], {'c1': step_num, 'u1': 0})
        else:
            return self.do_motion(self.motions['Soccer_Small_Step_Right'], {'c1': abs(step_num), 'u1': 0})


###########################################################################################
#
###########################################################################################

    def apply(self, action):
        if self._timer_permission_check():
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
            elif action['name'] == 'take_around_left':
                return self.do_motion(self.motions['Soccer_Take_Around_Left'] , {'c1': 1, 'u1': 0})

        return self.get_odometry(self.motions['Empty'], 0, 0, 0)

if __name__ == "__main__":
    m = Motion(True, False, False)
    print(m.apply({'name': 'turn', 'args':(0.2)}))
    print(m.apply({'name': 'walk', 'args':(0.3, 0.1)}))

