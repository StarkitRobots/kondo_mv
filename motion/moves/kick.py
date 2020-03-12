from move import Move
class Kick(Move):
    def __init__(self):
        pass

    def enter(self):
        pass

    def tick(self, side):
        if side == -1:
            return {'motion':'Soccer_Kick_Forward_Left_leg', 'args':{'c1': 0, 'u1': 0}}
        else:
            return {'motion':'Soccer_Kick_Forward_Right_leg', 'args':{'c1': 0, 'u1': 0}}
    def exit(self):
        pass