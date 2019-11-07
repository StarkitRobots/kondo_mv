import math

class Model:
    cameraPan = 0
    cameraTilt = 0

    def updateCameraPanTilt(self, cameraPan, cameraTilt):
        self.cameraPan = cameraPan
        self.cameraTilt = cameraTilt

    def setParams(self, A, h):
        self.A = A
        self.h = h

    def r2pic(self, xb, yb):

        xbi = xb * math.cos(self.cameraPan) + yb * math.sin(self.cameraPan)
        ybi = yb * math.cos(self.cameraPan) - xb * math.sin(self.cameraPan)

        if xbi <= 0:
            return "problem"

        alp = math.atan(self.h / xbi) + self.cameraTilt
        x = ybi / math.cos(alp) / math.sqrt(xbi**2 + self.h ** 2)
        y = math.tan(alp)
        u = x * self.A[0][0] + self.A[0][2]
        v = y * self.A[1][1] + self.A[1][2]
        return (int(u), int(v))

    def pic2r(self, u, v):
        x = (u - self.A[0][2]) / 1.0 / self.A[0][0]
        y = (v - self.A[1][2]) / 1.0 / self.A[1][1]
        alp = math.atan(y)
        bet = math.atan(x)
        balp = alp - self.cameraTilt
        bbet = bet + self.cameraPan

        if math.tan(balp) == 0:
            return "problem"

        xb = self.h / math.tan(balp) * math.cos(bbet)
        yb = self.h / math.tan(balp) * math.sin(bbet)

        return (yb, xb)
