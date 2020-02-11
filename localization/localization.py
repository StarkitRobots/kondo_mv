
from ParticleFilter import updatePF, ParticleFilter
import sys
from robot import Robot
from field import Field
sys.path.append('/')
from common import median
import json
import math
class Localization:
    def __init__(self, x, y, yaw, side):
        self.ballPosSelf = None
        self.ball_position = None
        self.robot_position = None
        self.localized = False
        self.seeBall = False
        self.side = False
        with open("localization/landmarks.json", "r") as f:
                landmarks = json.loads(f.read())
        #choice side
        if side:
            colors = []
            for goal_color in landmarks:
                colors.append(goal_color)
            neutral = landmarks[colors[0]]
            landmarks[colors[0]] = landmarks[colors[1]]
            landmarks[colors[1]] = neutral


        self.pf = ParticleFilter(Robot(x, y, yaw), Field("localization/parfield.json"), landmarks)

    def update(self, data):
        #self.ball_position = data["ball"]
        self.robot_position = updatePF(self.pf, data)
        if self.pf.consistency > 0.5:
            self.localized = True
        else:
            self.localized = False


    def update_ball(self, data):
        if len(data['ball']) != 0:
            #ballX = 0
            #ballY = 0
            #for el in data["ball"]:
            #    ballX+=el[0]
            #    ballY+=el[1]
            self.seeBall = True
            #self.ballPosSelf = (ballX/len(data['ball']), ballY/len(data['ball']))
            #bx = ballX/len(data['ball'])
            #by = ballY/len(data['ball'])

            self.ballPosSelf = median(data["ball"])
            #min(data["ball"], key=lambda x: x[0]**2 + x[1]**2)
            #print("ballPosSelf = ", self.ballPosSelf)

            bx = self.ballPosSelf[0]
            by = self.ballPosSelf[1]
            x_ball = self.pf.myrobot.x + bx*math.cos(-self.pf.myrobot.yaw) + by*math.sin(-self.pf.myrobot.yaw)
            y_ball = self.pf.myrobot.y - bx*math.sin(-self.pf.myrobot.yaw) + by*math.cos(-self.pf.myrobot.yaw)
            self.ball_position = (x_ball, y_ball)
            #self.ball_position = ()

            print("eto ball", self.ball_position)
        else:
            self.seeBall = False

    #self.robot_position = updatePF(self.pf, data):
        #return 0

    def move(self, x, y, yaw):
        self.pf.particles_move(x, y, yaw)

