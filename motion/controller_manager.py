import sys
sys.path.append('../lowlevel')
from kondo_controller import Rcb4BaseLib
from pyb import UART
import json, time, math

CHANNEL_WAIT_TIME = 10

class ControllerManager:
    def __init__(self):    
        self.kondo = Rcb4BaseLib()
        _uart = UART(1, 115200, parity=0, timeout=1000)
        self.kondo.open(_uart)
        print('ControllerManager: Controller acknowledge ', self.kondo.checkAcknowledge())
        with open("model/kal.json", "r") as f:
            self.servos = json.loads(f.read())

        # timer for motions
        self._motion_duration = 0.0
        self._motion_start_time = 0.0

###########################################################################################
# timer methods
###########################################################################################

    # False if controller is busy (playing motion).
    def _timer_permission_check(self):
        timer_check = self._motion_start_time + self._motion_duration <= time.ticks()
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
        self._motion_start_time = time.ticks()
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

    def set_servos(self, servo_data):
        bus = []
        for data in servo_data:
            pos = round(servo_data[data] / 180 * math.pi * 1698 + 7500)
            bus.append(
                self.kondo.ServoData(self.servos[data]['id'], self.servos[data]['sio'], pos))

        self.kondo.setServoPos(bus, 2)

