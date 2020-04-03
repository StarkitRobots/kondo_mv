import math

class Robot:
    def __init__(self, x = 0, y = 0, yaw = 0):
        self.x = x          # robot's x coordinate
        self.y = y          # robot's y coordinate
        self.yaw = yaw      # robot's angle

    def set_coord(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.yaw = float(new_orientation)

    def move(self, x, y, yaw):
        # turn, and add randomomness to the turning command
        orientation = self.yaw + float(yaw)
        if orientation < 0:
            orientation += (math.pi * 2)
        orientation %= (2 * math.pi)
        self.x += x*math.cos(self.yaw)
        self.y += x*math.sin(self.yaw)
        self.yaw = orientation
