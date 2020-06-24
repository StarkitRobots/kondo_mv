import math


class BallApproach:

    dist = 1
    turn_ang = 0.1 * math.pi

    def get_data(self, xr, yr, xb, yb, yaw):
        # xr, yr, yaw - coords of the robot in global system
        # xb, yb - coords of the ball in global system
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
        self.circle_radius = consts["CIRCLE_RADIUS"]
        self.goal_len = consts["GOAL_LEN"]
        self.wind_x = consts["WIND_X"]
        self.goal_pos = consts["GOAL_POS"]
        self.step_before_strike = consts["step_before_strike"]
        self.medium_dist = consts["medium_dist"]
        self.critical_lateral_step = consts["critical_lateral_step"]

    def get_diff(self):
        return self.xr - self.xb

    def find_trajectory(self):
        xr = self.xr
        yr = self.yr
        xb = self.xb
        yb = self.yb

        circle_radius = self.circle_radius
        goal_len = self.goal_len
        wind_x = self.wind_x
        goal_pos = self.goal_pos

        traj = []

        traj.append((xr, yr))

        # find kick point on the circle
        gbx = xb - wind_x                        # goal-ball x
        gby = yb - goal_pos - int(goal_len / 2)  # goal-ball y

        length_gb = math.sqrt(gbx**2 + gby**2)

        kpx = xb + circle_radius * gbx / length_gb  # kick point x
        kpy = yb + circle_radius * gby / length_gb  # kick point y

        traj.append((kpx, kpy))
        traj.append((wind_x, goal_pos + goal_len / 2))

        traj[1] = (xb, yb)

        self.wtraj = traj

        return traj

    # linear tranformation
    def lin_trans(self, W, shift, vec):
        res = [vec[i] - shift[i] for i in range(2)]
        res = [W[i][0] * res[0] + W[i][1] * res[1] for i in range(2)]
        return res

    # finding trajectory in local robot coords
    def convert_trajectory(self):
        traj = self.wtraj
        yaw = -self.yaw

        # shift,rot_mat - coefficients of the rotation transformation
        shift = traj[0]
        rot_mat = [[math.cos(yaw), math.sin(yaw)],
                   [-math.sin(yaw), math.cos(yaw)]]

        rtraj = [self.lin_trans(rot_mat, shift, point) for point in traj]
        self.rtraj = rtraj
        return rtraj

    def make_decision(self):

        rtraj = self.rtraj
        traj = self.wtraj
        # ang_thres2 - minimum angle between the parts of the trajectory
        ang_thres2 = self.ang_thres2
        yaw = -self.yaw

        # targ_vec - the vector from  center of the goal to the ball
        # targ_vec_ln - length of the targ_vec
        # norm - normal to targ_vec unit vector
        world_targ_vec = (traj[1][0] - traj[2][0], traj[1][1] - traj[2][1])
        targ_vec = (rtraj[2][0] - rtraj[1][0], rtraj[2][1] - rtraj[1][1])
        world_targ_vec_ln = math.sqrt(world_targ_vec[0] ** 2 +
                                      world_targ_vec[1] ** 2)
        targ_vec_ln = math.sqrt(targ_vec[0] ** 2 + targ_vec[1] ** 2)
        world_norm = (world_targ_vec[1] * 1.0 / world_targ_vec_ln,
                      world_targ_vec[0] * 1.0 / world_targ_vec_ln)

        # path - vector from robot to ball
        # pth_ln - length of path
        path = rtraj[1]
        pth_ln = math.sqrt(path[0] ** 2 + path[1] ** 2)
        self.dist = pth_ln

        # ang1 - angle between robot walk direction and robot-to-ball direction
        # ang2 - angle between the two parts of the trajectory
        # vec_prod - shows the orientation of the parts of the trajectory
        ang1 = math.acos(path[0] / pth_ln)
        ang2 = math.acos((targ_vec[0] * path[0] + targ_vec[1] * path[1]) /
                         targ_vec_ln / pth_ln)
        self.ang2 = ang2
        print("rtraj:", rtraj)
        vec_prod = targ_vec[1] * path[0] - targ_vec[0] * path[1]

        # calculating the circle, that smoothes the movement of the robot
        circle_rad = pth_ln / 2.0 / math.sin(ang2)
        self.circle_rad = circle_rad
        if vec_prod < 0:
            self.circle_center = (traj[1][0] - world_norm[0] * circle_rad,
                                  -traj[1][1] - world_norm[1] * circle_rad)
        elif vec_prod > 0:
            self.circle_center = (traj[1][0] + world_norm[0] * circle_rad,
                                  -traj[1][1] + world_norm[1] * circle_rad)

        # the current destination point on the circle
        vec_shift = circle_rad - circle_rad * math.cos(ang2)
        sup_point = (traj[0][0] / 2 + traj[1][0] / 2,
                     -traj[0][1] / 2 + -traj[1][1] / 2)
        s_c = self.circle_center
        l_vec = (s_c[0] - sup_point[0], s_c[1] - sup_point[1])
        l_vec_ln = math.sqrt(l_vec[0] ** 2 + l_vec[1] ** 2)
        l_vec_n = (l_vec[0] / l_vec_ln, l_vec[1] / l_vec_ln)

        if ang2 < math.pi / 2:
            move_point_w = (sup_point[0] - l_vec_n[0] * vec_shift,
                            -sup_point[1] + l_vec_n[1] * vec_shift)
        else:
            move_point_w = (sup_point[0] + l_vec_n[0] * vec_shift,
                            -sup_point[1] - l_vec_n[1] * vec_shift)

        self.move_p = move_point_w
        shift = traj[0]
        rot_mat = [[math.cos(yaw), math.sin(yaw)],
                   [-math.sin(yaw), math.cos(yaw)]]

        move_point_loc = self.lin_trans(rot_mat, shift, move_point_w)
        move_p_dist = math.sqrt(move_point_loc[0] ** 2 +
                                move_point_loc[1] ** 2)

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
                if ang2 > math.pi - ang_thres2:
                    return "lateral step", self.critical_lateral_step

                if move_point_loc[1] > 0:
                    return "walk", move_p_dist, ang3
                elif move_point_loc[1] < 0:
                    return "walk", move_p_dist, -ang3
