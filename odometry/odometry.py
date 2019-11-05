import math

class Kick:
    def __init__(self):
        pass
    
    def timer(self, c1):
        return 1000


class Turn:
    def __init__(self):
        pass
    
    def timer(self, c1):
        return 220 * c1 + 80


class SideStep:
    def __init__(self):
        pass
    
    def timer(self, c1):
        return 180 * c1 + 360

    def shift_x(self, c1, u1):
        return 0

    def shift_y(self, c1):
        return 11. * c1

class SideStepRight(SideStep):
    def __init__(self):
        self.id = 10

    def shift_y(self):
        return 

class SmallSideStep:
    def __init__(self):
        pass
    
    def timer(self, c1):
        return 1000

    def shift_x(self, c1, u1):
        return 0

    def shift_y(self, c1, side=1):
        return side * 3.3 * c1

class TurnAround:
    def __init__(self):
        pass
    
    def timer(self, c1):
        return 1000
