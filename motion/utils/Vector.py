import math

class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def norm(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self):
        norm = self.norm()
        self.x /= norm
        self.y /= norm
        self.z /= norm
        return self

class Quaternion(Vector):
    def __init__(self, x, y, z, w):
        super().__init__(x, y, z)
        self.w = w
    def normalize_vector(self):
        norm = super().norm()
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