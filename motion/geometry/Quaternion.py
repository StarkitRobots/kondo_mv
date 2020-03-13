import math

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    def normalize_vector(self):
        norm = math.sqrt(self.x**2 + self.y**2 + self.z**2)
        self.x /= norm
        self.y /= norm
        self.z /= norm
        return self

    def norm(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)

    def normalize(self):
        norm = self.norm()
        self.x /= norm
        self.y /= norm
        self.z /= norm
        self.w /= norm
        return self