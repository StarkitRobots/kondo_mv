class Move:
    def __init__(self):
        self.servos = {}
        self.name = ''
    
    def enter(self):
        data = []
        data.append(self.servos)
        return data

    def tick(self):
        data = []
        data.append(self.servos)
        return data

    def exit(self):
        data = []
        data.append(self.servos)
        return data