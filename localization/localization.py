from ParticleFilter import updatePF, ParticleFilter, Robot, Field
import json
class Localization:
    def __init__(self):
        self.ball_position = None
        self.robot_position = None
        self.localized = False
        self.seeBall = False
        self.robot = Robot()
        self.robot.set_coord(0.0, 0.0, 1.57)
        with open("localization/landmarks.json", "r") as f:
                landmarks = json.loads(f.read())
        self.pf = ParticleFilter(self.robot, Field("localization/parfield.json"), landmarks, sense_noise=0.2)

    def update(self, data):
        #self.ball_position = data["ball"]
        self.robot_position = updatePF(self.pf, data)
        return 0

    def move(self, x, y, yaw):
        self.pf.move(x,y, yaw)

