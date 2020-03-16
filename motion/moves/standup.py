from move import Move
class Standup(Move):
    def __init__(self):
        super().__init__()
        self.name = 'Standup'
        self.side = 0
    def enter(self):
        pass

    def tick(self):
        if self.side == -1:
            self.side = 0
            return {'motion':'Standup_front', 'args':{'c1': 0, 'u1': 0}}
        elif self.side == 1:
            self.side = 0
            return {'motion':'Standup_back', 'args':{'c1': 0, 'u1': 0}}
        else:
            return {}
    def exit(self):
        pass