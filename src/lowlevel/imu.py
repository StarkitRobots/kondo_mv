try:
    from machine import I2C
except ImportError:
    raise Exception("Try to import MicroPython library using Python3")
from .bno055 import BNO055, AXIS_P7
import time, math

CHANNEL_WAIT_TIME = 10

class IMU():
    def __init__(self):
        self.i2c = I2C(2)
        self.imu = BNO055(self.i2c, mode=0x08)

        #init position yaw, roll, pitch
        self.init_yaw, self.init_roll, self.init_pitch = self.tare()  

        self.yaw = self.roll = self.pitch = 0

    def tare(self):
        self.init_yaw, self.init_roll, self.init_pitch = self.get_transformed_data() 
        return self.init_yaw, self.init_roll, self.init_pitch
    
    @staticmethod
    def _change_orientation(yaw, roll, pitch):
        return 360 - yaw, 360 - roll, 360 - pitch
    
    @staticmethod
    def to_radians(yaw, roll, pitch):
        return yaw / 180 * math.pi, roll / 180 * math.pi, pitch / 180 * math.pi

    def get_raw_data(self):
        try:
            yaw, roll, pitch = self.imu.euler()
        except OSError:
            time.sleep(CHANNEL_WAIT_TIME)
            yaw, roll, pitch = self.imu.euler()
        return yaw, roll, pitch

    def get_transformed_data(self):
        yaw, roll, pitch = self.get_raw_data()
        yaw, roll, pitch = self._change_orientation(yaw, roll, pitch)
        if yaw > 180: 
            yaw -= 360
        if roll > 180: 
            roll -= 360
        if pitch > 180: 
            pitch -= 360
        return self.to_radians(yaw, roll, pitch)

    def update(self):
        yaw, roll, pitch = self.get_transformed_data()
        
        self.yaw = yaw - self.init_yaw
        self.roll = roll - self.init_roll
        self.pitch = pitch - self.init_pitch
        return self.yaw, self.roll, self.pitch 

    def get_yaw(self):
        self.update()
        return self.yaw

    def get_roll(self):
        self.update()
        return self.roll

    def get_pitch(self):
        self.update()
        return self.pitch
