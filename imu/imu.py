from machine import I2C
import sys
sys.path.append('../lowlevel')
from bno055 import BNO055, AXIS_P7
import time


class IMU():
    #def __init__(self, init_angle = 90.0):
    #    self.i2c = I2C(2)
    #    self.imu = BNO055(self.i2c)
    #    self.init_angle = init_angle
    #    self.init_yaw, self.init_roll, self.init_pitch = self.imu.euler()  #init position yaw, roll, pitch
    #    self.yaw, self.roll, self.pitch = self.imu.euler()
    #    #self.yaw = (self.yaw - self.init_yaw  + self.init_angle)%360
    #    self.yaw = self.init_angle
    #    print ('init yaw, yaw', self.init_yaw, self.yaw)

    def __init__(self, init_angle = 90.0):
        self.i2c = I2C(2)
        self.imu = BNO055(self.i2c)
        self.init_angle = init_angle

        self.init_yaw, _, _ = self.imu.euler()  #init position yaw, roll, pitch
        #self.init_yaw *= -1

        self.yaw = self.init_yaw#self.init_angle

        #print ('init yaw, yaw', self.init_yaw, self.yaw)

    #def update(self):
    #    yaw, roll, pitch = self.imu.euler()
    #    #yaw = self.yaw - (yaw - self.init_yaw)
    #    self.yaw = (self.yaw - yaw + 360)%360
    #    print('self.yaw, yaw', self.yaw, yaw)
    #    return yaw

    def update(self):
        #self.yaw - current yaw
        #yaw_diff - yaw_difference))0)

        yaw, _, _ = self.imu.euler()
        yaw *= -1

        #print('yaw imu, self.yaw, init yaw', yaw, self.yaw, self.init_yaw)

        yaw_diff = self.yaw - yaw
        #self.yaw_diff = yaw_diff + self.

        self.yaw = (self.yaw - yaw_diff + self.init_angle + 360)%360

        #print('self.yaw, yaw', self.yaw, yaw_diff)
        return self.yaw
