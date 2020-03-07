import math
import json


class BallApproach:

    dist = 1
    turn_ang = 0.1 * math.pi

    def get_data(self, xr, yr, xb, yb, yaw):
        self.xr = xr
        self.yr = yr
        self.xb = xb
        self.yb = yb
        self.yaw = yaw


    def set_constants(self, consts):
        self.max_step = consts["max_step"]
        self.min_dist = consts["min_dist"]
        self.ang_thres1 = consts["ang_thres1"]
        self.ang_thres2 = consts["ang_thres2"]
        self.CIRCLE_RADIUS = consts["CIRCLE_RADIUS"]
        self.GOAL_LEN = consts["GOAL_LEN"]
        self.WIND_X = consts["WIND_X"]
        self.GOAL_POS = consts["GOAL_POS"]
        self.step_before_strike = consts["step_before_strike"]
        self.medium_dist = consts["medium_dist"]
        self.critical_lateral_step = consts["critical_lateral_step"]


    def get_diff(self):
        return self.xr - self.xb

    def find_trajectory (self):
        xr = self.xr
        yr = self.yr
        xb = self.xb
        yb = self.yb
        #max_step = self.max_step
        CIRCLE_RADIUS = self.CIRCLE_RADIUS
        GOAL_LEN = self.GOAL_LEN
        WIND_X = self.WIND_X
        GOAL_POS = self.GOAL_POS

        traj = []

        traj.append ((xr, yr))

        #-----------------------------------------------------------
        #find starting point on the circle

        xbr = xb - xr #x ball relative
        ybr = yb - yr #y ball relative

        r    = CIRCLE_RADIUS
        leng = math.sqrt (xbr**2 + ybr**2)

        #beta  = math.asin (ybr / leng)
        #alpha = math.asin (r / leng)

        #if (xb < xr):# + CIRCLE_RADIUS):
        #   sx = 0
        #   sy = 0

        #   if (yr + (yr - GOAL_POS - int (GOAL_LEN / 2)) * (xr - xb) / (WIND_X - xb) > yb):
        #       sx = - leng * math.cos (alpha + beta) * math.cos (alpha) + xr
        #       sy = leng * math.sin (alpha + beta) * math.cos (- alpha) + yr

        #   else:
        #       alpha = - alpha
        #
        #       sx = - leng * math.cos (alpha + beta) * math.cos (alpha) + xr
        #       sy = leng * math.sin (alpha + beta) * math.cos (- alpha) + yr

        #   traj.append ((sx, sy))

        #-----------------------------------------------------------
        #find kick point on the circle
        gbx = xb - WIND_X                        #goal-ball x
        gby = yb - GOAL_POS - int (GOAL_LEN / 2) #goal-ball y

        length_gb =  math.sqrt (gbx**2 + gby**2)

        kpx = xb + CIRCLE_RADIUS * gbx / length_gb #kick point x
        kpy = yb + CIRCLE_RADIUS * gby / length_gb #kick point y

        traj.append ((kpx, kpy))

        #move with proper steps on the circle

        #if (xr < xb):
        #    traj.append ((xr, yr))
        #    traj.append ((xb - CIRCLE_RADIUS, yb))

        #else:
        #    if (yr < yb):
        #    else:

        traj.append ((WIND_X, GOAL_POS + GOAL_LEN / 2))

        traj[1] = (xb, yb )

        self.wtraj = traj

        return traj

    def lin_trans(self, W, shift, vec):
        res = [vec[i] - shift[i] for i in range(2)]
        res = [W[i][0] * res[0] + W[i][1] * res[1] for i in range(2)]
        return res


    def convert_trajectory(self):
        traj = self.wtraj
        yaw = -self.yaw

        # shift,rotmat - coefficients of the rotation transformation
        shift = traj[0]
        rotmat = [[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]]

        rtraj = [self.lin_trans(rotmat, shift, point) for point in traj]
        self.rtraj = rtraj
        return rtraj


    def make_decision(self):

        rtraj = self.rtraj
        traj = self.wtraj
        min_dist = self.min_dist
        ang_thres1 = self.ang_thres1 # - minimum allowed angle between robot walk direction and robot-to-ball direction
        ang_thres2 = self.ang_thres2 # - minimum allowed angle between the parts of the trajectory
        yaw = -self.yaw

        # targvec - the vector from  center of the goal to the ball
        # tv_ln - length of the targvec
        # norm - the vector normal to the targvec with the length taken from data.json
        wtargvec = (traj[1][0] - traj[2][0], traj[1][1] - traj[2][1])
        targvec = (rtraj[2][0] - rtraj[1][0], rtraj[2][1] - rtraj[1][1])
        wtv_ln = math.sqrt(wtargvec[0] ** 2 + wtargvec[1] ** 2)
        tv_ln = math.sqrt(targvec[0] ** 2 + targvec[1] ** 2)
        #norm = (self.step_before_strike * targvec[1] * 1.0 / tv_ln, -self.step_before_strike * targvec[0] * 1.0 / tv_ln)
        #wnorm = (wtargvec[1] * 1.0 / wtv_ln, wtargvec[0] * 1.0 / wtv_ln)

        # path - vector from robot to ball
        # pth_ln - length of path
        path = rtraj[1]
        pth_ln = math.sqrt(path[0] ** 2 + path[1] ** 2)
        self.dist = pth_ln

        # ang1 - angle between robot walk direction and robot-to-ball direction
        # ang2 - angle between the two parts of the trajectory
        ang1 = math.acos(path[0] / pth_ln)
        ang2 = math.acos((targvec[0] * path[0] + targvec[1] * path[1]) / tv_ln / pth_ln)
        self.ang2 = ang2
        print("rtraj:", rtraj)
        vec_prod = targvec[1] * path[0] - targvec[0] * path[1]

        # calculating the proprties of the circle, that smoothes the movement of the robot
        R = pth_ln / 2.0 / math.sin(ang2)
        self.R = R
        circ_path = R * 2.0 * ang2
        #if vec_prod < 0:
        #    self.circle_center = (traj[1][0] - wnorm[0] * R, -traj[1][1] - wnorm[1] * R)
        #elif vec_prod > 0:
        #    self.circle_center = (traj[1][0] + wnorm[0] * R, -traj[1][1] + wnorm[1] * R)

        # calculating the current point on the circle the robot should go towards
        vec_shift = R - R * math.cos(ang2)
        sup_point = (traj[0][0] / 2 + traj[1][0] / 2, -traj[0][1] / 2 + -traj[1][1] / 2)
        #s_c = self.circle_center
        l_vec = (s_c[0] - sup_point[0], s_c[1] - sup_point[1])
        l_vec_ln = math.sqrt(l_vec[0] ** 2 + l_vec[1] ** 2)
        l_vec_n = (l_vec[0] / l_vec_ln, l_vec[1] / l_vec_ln)

        if ang2 < math.pi / 2:
            move_point_w = (sup_point[0] - l_vec_n[0] * vec_shift, -sup_point[1] + l_vec_n[1] * vec_shift)
        else:
            move_point_w = (sup_point[0] + l_vec_n[0] * vec_shift, -sup_point[1] - l_vec_n[1] * vec_shift)
        
        self.move_p = move_point_w
        shift = traj[0]
        rotmat = [[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]]
        move_point_loc = self.lin_trans(rotmat, shift, move_point_w)
        move_p_dist = math.sqrt(move_point_loc[0] ** 2 + move_point_loc[1] ** 2)

        ang3 = math.acos(move_point_loc[0] / move_p_dist)


        # making the decision, based on the distance and angles
        ball_dist = math.sqrt((self.xb-self.xr)**2 + (self.yb-self.yr)**2)
        # print("ball dist approach = ", ball_dist)
        if (ball_dist < self.min_dist):
            if ang2 > ang_thres2:
                if vec_prod > 0:
                    return "take around right", 1
                else:
                    return "take around left", -1

            if path[1] > 0:
                return "left kick", -1
            else:
                return "right kick", 1

        else:
            if pth_ln > self.medium_dist:

                if path[1] > 0:
                    return "walk", self.dist, ang1
                else:
                    return "walk", self.dist, -ang1

            else:
                if ang2 > ang_thres2:
                    return "lateral step", self.critical_lateral_step

                if move_point_loc[1] > 0:
                    return "walk", move_p_dist, ang3
                elif move_point_loc[1] < 0:
                    return "walk", move_p_dist, -ang3
