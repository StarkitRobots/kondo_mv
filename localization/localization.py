from ParticleFilter import updatePF, ParticleFilter, Robot, Field
import json
class Localization:
    def __init__(self):
        self.ballPosSelf = None
        self.ball_position = None
        self.robot_position = None
        self.localized = False
        self.seeBall = False
        self.robot = Robot()
        self.robot.set_coord(0.0, 0.0, 0.0)
        with open("localization/landmarks.json", "r") as f:
                landmarks = json.loads(f.read())
        self.pf = ParticleFilter(self.robot, Field("localization/parfield.json"), landmarks, sense_noise=1.0)

    def update(self, data):
        #self.ball_position = data["ball"]
        self.robot_position = updatePF(self.pf, self.robot, data)
        return 0

    def end_of_loc(self):
        self.pf.logs.close()
