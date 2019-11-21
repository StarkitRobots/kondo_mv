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
        # xb, yb - coords of the ball in the robot system
        
        # rotating the coordinate system by the cameraPan
        # xbi,ybi - coords of the ball in the same system after rotation
        xbi = xb * math.cos(self.cameraPan) + yb * math.sin(self.cameraPan)
        ybi = yb * math.cos(self.cameraPan) - xb * math.sin(self.cameraPan)

        # taking into account the case when the ball is behind the camera
        if xbi <= 0:
            raise Exception('ball behind the camera')

        # alp - no real sense, used just for simplification
        # x,y - coords of the ball in the coordinate system, parallel to the camera screen
        # u,v - pixel coords of the ball in the screen system
        alp = math.atan(self.h / xbi) + self.cameraTilt
        x = ybi / math.cos(alp) / math.sqrt(xbi**2 + self.h ** 2)
        y = math.tan(alp)
        u = x * self.A[0][0] + self.A[0][2]
        v = y * self.A[1][1] + self.A[1][2]
        return (int(u), int(v))

    def pic2r(self, u, v):
        # u,v - pixel coords of the ball in the screen system

        # x,y - coords of the ball in the coordinate system, parallel to the camera screen
        x = (float(u) - self.A[0][2]) / self.A[0][0]
        y = (float(v) - self.A[1][2]) / self.A[1][1]
    
        # alp,bet - vertical and horizontal angles of the ball radius-vector relative to the camera 0,0 direction
        alp = math.atan(y)
        bet = math.atan(x)

        # robot_alp,robot_bet - vertical and horizontal angles of the ball radius-vector relative to the robot body direction
        robot_alp = alp - self.cameraTilt
        robot_bet = bet - self.cameraPan

        # the case when the ball is in the horizon
        if robot_alp <= 0:
            raise Exception('ball above the horizon')

        # xb,yb - the coords of the ball in the robot system
        xb = self.h / math.tan(robot_alp) * math.cos(robot_bet)
        yb = self.h / math.tan(robot_alp) * math.sin(robot_bet)

        return (xb, -yb)