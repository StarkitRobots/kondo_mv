from ParticleFilter import updatePF, ParticleFilter
from robot import Robot
from field import Field
import json
class Localization:
    def __init__(self, x, y, yaw):
        self.ballPosSelf = None
        self.ball_position = None
        self.robot_position = None
        self.localized = False
        self.seeBall = False
        with open("localization/landmarks.json", "r") as f:
                landmarks = json.loads(f.read())
        self.pf = ParticleFilter(Robot(x, y, yaw), Field("localization/parfield.json"), landmarks)

    def update(self, data):
        #self.ball_position = data["ball"]
        self.robot_position = updatePF(self.pf, data)


    def update_ball(self, data):
        if len(data['ball']) != 0:
            ballX = 0
            ballY = 0
            for el in data["ball"]:
                ballX+=el[0]
                ballY+=el[1]
            self.seeBall = True
            self.ballPosSelf = (ballX/len(data['ball']), ballY/len(data['ball']))
            #print("eto ball", self.ballPosSelf)
        else:
            self.seeBall = False

    #self.robot_position = updatePF(self.pf, data):
        #return 0

    def move(self, x, y, yaw):
        self.pf.particles_move(x,y, yaw)

