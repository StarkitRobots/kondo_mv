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

            "Standup_front" : {
                "id"        : 6,
                "time"      : (lambda c1, u1: 5000),
                "shift_x"   : (lambda c1, u1: 0),
                "shift_y"   : (lambda c1, u1: 0),
                "shift_turn": (lambda c1, u1: 0)
                },
            "Standup_back" : {
                "id"        : 7,
                "time"      : (lambda c1, u1: 5000),
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

    def get_motion_shift_x(self, motion, c1, u1):
        return self.motions['motion']['shift_x'](c1, u1)

    def get_motion_shift_y(self, motion, c1, u1):
        return self.motions['motion']['shift_y'](c1, u1)

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

    def get_motion_id(self, motion):
        return self.motions[motion]['id']

    def get_timer(self, motion, c1, u1):
        return self.motions[motion]['time'](c1, u1)

    def set_imu_yaw(self):
        self.imu_yaw_pos = self.imu.get_yaw()

    def get_motion_odometry(self, motion, c1, u1):

            return {'shift_x': self.get_motion_shift_x(motion, c1, u1), 
                'shift_y': self.get_motion_shift_y(motion, c1, u1), 
                'shift_yaw': self.get_shift_yaw()}

    def get_shifts(self):
        return {'shift_x': self.get_shift_x(), 
                'shift_y': self.get_shift_y(), 
                'shift_yaw': self.get_shift_yaw()}
