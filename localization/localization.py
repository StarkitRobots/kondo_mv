import math
import json
import sys
from pyb import Pin

sys.path.append('/')

from robot import Robot
from field import Field
from common import median
from ParticleFilter import updatePF, ParticleFilter

pin9 = Pin('P9', Pin.IN, Pin.PULL_UP)
pin3 = Pin('P3', Pin.IN, Pin.PULL_UP)
pin2 = Pin('P2', Pin.IN, Pin.PULL_UP)

class Localization:

    def __init__(self, x, y, yaw, side):
        ala = 0
        side = False
        ala = 1
        while(ala == 0):
            if (pin9.value() == 0):   # нажатие на кнопку на голове
                ala = 1
                side = True
                print("I will attack blue goal")
                break
        if (pin3.value() == 0):
            ala = 1
            side = False
            print("I will attack yellow goal")
            break

        if (pin2.value() == 0):
            ala = 1
            side = False
            print("I will attack yellow goal")
            break

        self.ballPosSelf = None
        self.ball_position = None
        self.robot_position = None
        self.localized = False
        self.seeBall = False
        self.side = False
        with open("localization/landmarks.json", "r") as f:
            landmarks = json.loads(f.read())
        # choice side
        if side:
            colors = []
            for goal_color in landmarks:
                colors.append(goal_color)
            neutral = landmarks[colors[0]]
            landmarks[colors[0]] = landmarks[colors[1]]
            landmarks[colors[1]] = neutral

        self.pf = ParticleFilter(Robot(x, y, yaw), Field(
            "localization/parfield.json"), landmarks)

    def update(self, data):
         self.robot_position = updatePF(self.pf, data)
        if self.pf.consistency > 0.5:
            self.localized = True
        else:
            self.localized = False

    def update_ball(self, data):
        if len(data['ball']) != 0:
            self.seeBall = True

            self.ballPosSelf = median(data["ball"])
            #min(data["ball"], key=lambda x: x[0]**2 + x[1]**2)
            #print("ballPosSelf = ", self.ballPosSelf)

            bx = self.ballPosSelf[0]
            by = self.ballPosSelf[1]
            x_ball = self.pf.myrobot.x + bx * \
                math.cos(self.pf.myrobot.yaw) - by * \
                math.sin(self.pf.myrobot.yaw)
            y_ball = self.pf.myrobot.y + bx * \
                math.sin(self.pf.myrobot.yaw) + by * \
                math.cos(self.pf.myrobot.yaw)
            self.ball_position = (x_ball, y_ball)
            #self.ball_position = ()

            print("eto ball", self.ball_position)
        else:
            self.seeBall = False

    def move(self, x, y, yaw):
        self.pf.particles_move(x, y, yaw)