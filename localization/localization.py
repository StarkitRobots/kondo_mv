from ParticleFilter import updatePF, ParticleFilter, Robot, Field
import json
import math
class Localization:
    def __init__(self, x, y, yaw):
        self.postPan = []
        self.ballPosSelf = None
        self.ball_position = None
        self.robot_position = None
        self.localized = False
        self.seeBall = False
        self.robot = Robot()
        self.robot.set_coord(x, y, yaw)
        with open("localization/landmarks.json", "r") as f:
                landmarks = json.loads(f.read())
        self.pf = ParticleFilter(self.robot, Field("localization/parfield.json"), landmarks, sense_noise=0.5)

    def update(self, data):
        #self.ball_position = data["ball"]
        for i in range(5):
            self.robot_position = updatePF(self.pf, data)
        self.localized = True
        x_ball = self.robot_position[0] - self.ballPosSelf[0]*math.cos(-self.robot_position[2])
        + self.ballPosSelf[1]*math.sin(-self.robot_position[2])
        y_ball = self.robot_position[1] - self.ballPosSelf[0]*math.sin(-self.robot_position[2])
        + self.ballPosSelf[1]*math.cos(-self.robot_position[2])
        self.ball_position = (x_ball, y_ball)
        return 0

    def update_posts(self, data, posts):
        if len(data[posts]) != 0:
            self.posts = data[posts]
            self.postPan = []
            for post in self.posts:
                self.postPan.append(math.atan(post[1]/post[0]))

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
        self.pf.move(x,y, yaw)

