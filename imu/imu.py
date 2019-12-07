from machine import I2C
from bno055 import BNO055, AXIS_P7
import time
import sensor, image, time



class IMU():
    def __init__(self):
        self.i2c = I2C(2)
        self.imu = BNO055(self.i2c)
        self.init_yaw, self.init_roll, self.init_pitch = self.imu.euler()  #init position yaw, roll, pitch

        # self.current_yaw = self.init_yaw
        #self.current_yaw, self.current_roll, self.current_pitch = imu.euler() # current position yaw, roll, pitch

        #change zero position 360 --- 0

        self.init_yaw_our = 90.0
        # self.init_yaw = self.init_yaw_our

        #  self.init_roll = self.init.roll1 - 360
        # self.init_pitch = self.init.pitch1 - 360


    def update(self):
        self.a_yaw, self.b_roll, self.c_pitch = self.imu.euler()
        self.current_yaw = (-self.a_yaw - self.init_yaw - 360 + self.init_yaw_our)%360

        #self.current_roll = self.b_roll - 360
        #self.current_pitch = self.c_pitch - 360
        time.sleep(2500)
    def change(self):
        self.a1_yaw, self.b1_roll, self.c1_pitch = self.imu.euler()
        self.current_yaw2 = (-self.a1_yaw - self.init_yaw - 360 + self.init_yaw_our)%360
       # self.shange_yaw = self.a1_yaw  - self.current_yaw

        #global change
        self.change_position_yaw = self.current_yaw2 - self.current_yaw
        #self.change_position_roll = self.current_roll - self.init_roll
        #self.change_position_pitch = self.current_pitch - self.init_pitch


      #  time.sleep(2500)