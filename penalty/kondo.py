#coding: UTF-8
from kondo_controller import Rcb4BaseLib
import utime
import kondo_conf
from pyb import UART

# Main constants of Kondo
ROTATION_ANGLE = kondo_conf.ROTATION_ANGLE
STEP_DISTANCE = kondo_conf.STEP_DISTANCE
DEADLOCK_TIME = kondo_conf.DEADLOCK_TIME


class Kondo:
    def __init__(self, debug=False):
        self.rcb4 = Rcb4BaseLib()
        self.state = -1
        self.debug = debug
        self.state_dict = kondo_conf.movements

    def init(self, device):
        if not self.debug:
            if isinstance(device, str):
                print("It's a special OpenMV lib")
            else:
                try:
                    print("trying to connect to " + str(device))
                    self.rcb4.open(device)
                    print("Connection to " + str(device) + " established")
                    self.state = 0
                except:
                    print("UART open error")

    def get_state(self):
        if self.state < 0:
            print("Do Kondo.init() function")
            return -1
        else:
            print(self.state)
            motion_num = self.rcb4.getMotionPlayNum()
            print(motion_num)
            if motion_num < 0:
                print('motion get error', motion_num)
                return -1
        return motion_num

    def run_motion(self, motion):
        if not self.debug:
            if motion < 0:
                print('Motion error')
            else:
                self.rcb4.motionPlay(motion)
                deadlock_time = utime.time()
                while True:
                    self.state = self.get_state()
                    if self.state <= 0:
                        break
                    utime.sleep(0.1)
                    if deadlock_time - utime.time() > DEADLOCK_TIME:
                        print("Deadlock")
                        break
            if self.state == 0:
                return 0
            else:
                return -1
        else:
            return 0

    def walk(self, step_num):
        if step_num > 0:
            for i in range(step_num):
                if self.run_motion(self.state_dict['step_forward']) == -1:
                    return -1
            return 0
        elif step_num < 0:
            for i in range(abs(step_num)):
                if self.run_motion(self.state_dict['step_backward']) == -1:
                    return -1
            return 0
        return 0

    def lateral_walk(self, step_num):
        if step_num > 0:
            for i in range(step_num):
                if self.run_motion(self.state_dict['lateral_step_right']) == -1:
                    return -1
            return 0
        elif step_num < 0:
            for i in range(abs(step_num)):
                if self.run_motion(self.state_dict['lateral_step_left']) == -1:
                    return -1
            return 0
        return 0

    def small_walk(self, step_num):
        if step_num > 0:
            for i in range(step_num):
                if self.run_motion(self.state_dict['small_step_forward']) == -1:
                    return -1
            return 0
        elif step_num < 0:
            for i in range(abs(step_num)):
                if self.run_motion(self.state_dict['small_step_backward']) == -1:
                    return -1
            return 0
        return 0

    def turn(self, angle):
        if angle > 0:
            for i in range(angle // ROTATION_ANGLE):
                print("angle", angle)
                if self.run_motion(self.state_dict['turn_right']) == -1:
                    return -1
            return 0
        elif angle < 0:
            for i in range(abs(angle) // ROTATION_ANGLE):
                if self.run_motion(self.state_dict['turn_left']) == -1:
                    return -1
            return 0
        return 0

    def tilt(self):
        if self.run_motion(self.state_dict['tilt']) == -1:
            return -1
        else:
            return 0

    def straighten(self):
        if self.run_motion(self.state_dict['straighten']) == -1:
            return -1
        else:
            return 0

    def triple_jump(self):
        if self.run_motion(self.state_dict['triple_jump']) == -1:
            return -1
        else:
            return 0

    def body_turn(self, angle):
        if angle > 0:
            #for i in range(angle // ROTATION_ANGLE):
            if self.run_motion(self.state_dict['body_turn_right']) == -1:
                return -1
            #return 0
        elif angle < 0:
            #for i in range(abs(angle // ROTATION_ANGLE)):
            if self.run_motion(self.state_dict['body_turn_left']) == -1:
                return -1
            #return 0
        return 0

    def archery_stand(self):
        if self.run_motion(self.state_dict['archery_stand']) == -1:
            return -1
        else:
            return 0

    def grab_arrow(self):
        if self.run_motion(self.state_dict['grab_arrow']) == -1:
            return -1
        else:
            return 0

    def stretch_bow(self):
        if self.run_motion(self.state_dict['stretch_bow']) == -1:
            return -1
        else:
            return 0

    def shoot(self):
        if self.run_motion(self.state_dict['shoot']) == -1:
            return -1
        else:
            return 0

    def head_up(self):
        if self.run_motion(self.state_dict['head_up']) == -1:
            return -1
        else:
            return 0

    def head_down(self):
        if self.run_motion(self.state_dict['head_down']) == -1:
            return -1
        else:
            return 0

    def head_left(self):
        if self.run_motion(self.state_dict['head_left']) == -1:
            return -1
        else:
            return 0

    def head_right(self):
        if self.run_motion(self.state_dict['head_right']) == -1:
            return -1
        else:
            return 0

    def __del__(self):
        print("Closing connection")
        self.rcb4.close()
        self.state = -1


if __name__ == "__main__":
   kondo = Kondo()
   uart = UART(1, 115200, timeout=1000, parity=0)
   kondo.init(uart)
   print(kondo.rcb4.getMotionPlayNum())
   #kondo.walk(1)
