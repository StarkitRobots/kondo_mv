import time
import math
try:
    from pyb import UART
except ImportError:
    raise Exception("Try to import MicroPython library using Python3")
from .rcb4 import Rcb4BaseLib

CHANNEL_WAIT_TIME = 10

class KondoController:
    def __init__(self, servos):    
        self.kondo = Rcb4BaseLib()
        _uart = UART(1, 115200, parity=0, timeout=1000)
        self.kondo.open(_uart)

        self.servos = servos

        # timer for motions
        self._motion_duration = 0.0
        self._motion_start_time = 0.0

###########################################################################################
# timer methods
###########################################################################################

    # False if controller is busy (playing motion).
    def _timer_permission_check(self):
        try:
            timer_check = self._motion_start_time + self._motion_duration <= time.ticks()
        except AttributeError:
            raise Exception("Trying to run controller not on OpenMV")
        motion_finished = False
        while not (motion_finished and timer_check):
            try:
                time.sleep(100)
                current_motion = self.kondo.getMotionPlayNum()
            except OSError:
                time.sleep(CHANNEL_WAIT_TIME)
                current_motion = self.kondo.getMotionPlayNum()
            motion_finished = current_motion == 0 or current_motion == 1 or current_motion == 2
        return True

    def _get_timer_duration(self, motion, args):
        return motion['time'](args['c1'], args['u1'])

    def _set_timer(self, duration):
        self._motion_duration = duration  
        try:
            self._motion_start_time = time.ticks()
        except AttributeError:
            raise Exception("Trying to run controller not on OpenMV")
        time.sleep(int(duration))

    # basic method of any motion appliance
    def do_motion(self, target_motion, args=None):
        c1 = 0
        u1 = 0
        if self._timer_permission_check():
            self.current_motion = target_motion
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

    def check_acknowledge(self):
        return self.kondo.checkAcknowledge()

    def set_servos(self, servo_data):
        bus = []
        for servo in servo_data:
            pos = round(servo_data[servo] * 1698 + 7500)
            bus.append(
                self.kondo.ServoData(self.servos[servo]['id'], self.servos[servo]['sio'], pos))

        self.kondo.setServoPos(bus, 2)

    def falling_test(self):
        falling_flag = 0
        pitch = self.kondo.getAdData(3)
        roll = self.kondo.getAdData(4)
        if pitch < 200:
            falling_flag = 1     # on stomach
        if pitch > 450:
            falling_flag = -1    # face up
        if roll > 400:
            falling_flag = -2    # on right side
        if roll < 160:
            falling_flag = 2     # on left side

        return falling_flag

