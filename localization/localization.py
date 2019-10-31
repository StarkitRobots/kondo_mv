from ParticleFilter import updatePF

class Localization:
    def __init__(self):
        self.ball_position = None
        self.robot_position = None

    def update(self, data):
        self.ball_position = data["ball"]
        self.robot_position = updatePF(data)
        return 0