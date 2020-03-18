from .move import Move

class Kick(Move):
    def __init__(self):
        super().__init__()
        self.name = 'kick'
        self.side = 0
    def enter(self):
        pass

    def tick(self):
        if self.side == -1:
            self.side = 0
            return {'motion':'Soccer_Kick_Forward_Left_leg', 'args':{'c1': 0, 'u1': 0}}
        elif self.side == 1:
            self.side = 0
            return {'motion':'Soccer_Kick_Forward_Right_leg', 'args':{'c1': 0, 'u1': 0}}
        else:
            return {}
    def exit(self):
        pass