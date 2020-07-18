import math

class Odometry:
    def __init__(self, imu=None):
        if imu is None:
            print('No imu mode')
        else:
            self.imu = imu
        self.imu_yaw_pos = None
        self.shift_x = 0
        self.shift_y = 0
        self.shift_yaw = 0

    def get_shift_x(self):
        return self.shift_x

    def get_shift_y(self):
        return self.shift_y

    def get_shift_yaw(self, motion=None):
        if self.imu_yaw_pos is None:
            if motion is not None:
                print('IMU was not set before the motion {0}. 0 is returned'.format(motion))
            else:
                print('IMU was not set')
            return 0
        else:
            self.shift_yaw = self.imu.get_yaw - self.imu_yaw_pos
            if self.shift_yaw > 180:
                self.shift_yaw -= 360
            if self.shift_yaw < -180:
                self.shift_yaw += 360
        return self.shift_yaw

    def set_imu_yaw(self):
        self.imu_yaw_pos = self.imu.get_yaw()

    def get_shifts(self):
        return {'shift_x': self.get_shift_x(), 
                'shift_y': self.get_shift_y(), 
                'shift_yaw': self.get_shift_yaw()}
