import math

class Model:
    cameraPan = 0
    cameraTilt = 0

    def updateCameraPanTilt(self, cameraPan, cameraTilt):
        self.cameraPan = cameraPan
        self.cameraTilt = cameraTilt

    def setParams(self, A, h, k_coefs, p_coefs):
        self.A = A
        self.h = h
        self.k_coefs = k_coefs
        self.p_coefs = p_coefs

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
        
        # applying the lense distortion-fix formula
        r_sq = x ** 2 + y ** 2
        coef_numerator = (1 + self.k_coefs[0] * r_sq + self.k_coefs[1] * (r_sq ** 2) + self.k_coefs[2] * (r_sq ** 3))
        coef_denominator = (1 + self.k_coefs[3] * r_sq + self.k_coefs[4] * (r_sq ** 2) + self.k_coefs[5] * (r_sq ** 3))
        coef = coef_numerator / coef_denominator
        
        x_cor = x * coef + 2 * self.p_coefs[0] * x * y + self.p_coefs[1] * (r_sq + 2 * x ** 2)
        y_cor = y * coef + 2 * self.p_coefs[1] * x * y + self.p_coefs[0] * (r_sq + 2 * y ** 2)
        
        u = -x_cor * self.A[0][0] + self.A[0][2]
        v = y_cor * self.A[1][1] + self.A[1][2]
        return (int(u), int(v))
    
    
    def pic2r(self, u, v):
        # u,v - pixel coords of the ball in the screen system

        # x,y - coords of the ball in the coordinate system, parallel to the camera screen
        x = (float(u) - self.A[0][2]) / self.A[0][0]
        y = (float(v) - self.A[1][2]) / self.A[1][1]
    
        # alp,bet - vertical and horizontal angles of the ball radius-vector relative to the camera 0,0 direction
        alp = math.atan(y)
        bet = math.atan(x)

        # robot_alp- vertical angle of the ball radius-vector relative to the robot body direction
        robot_alp = alp - self.cameraTilt
        
        # the case when the ball is in the horizon
        if robot_alp <= 0:
            raise Exception('ball above the horizon')
        
        # xb, yb - coordinates of the ball in the system, which is turned by cameraPan relative to the robot system               
        xb = self.h / math.tan(robot_alp)
        yb = math.tan(bet) * math.sqrt(self.h ** 2 + xb ** 2)
        
        # xb_r, yb_r - coordinates of the ball in the robot system   
        xb_r = xb * math.cos(self.cameraPan) + yb * math.sin(self.cameraPan)
        yb_r = yb * math.cos(self.cameraPan) - xb * math.sin(self.cameraPan)

        return (xb_r, -yb_r)