import sys
import math
import json

from ball_approach import BallApproach


class Strategy:
    def __init__(self):
        self.turn_counterFF = 0  # counter for localized=False and seeBall=False 
        self.turn_counterTF = 0  # counter for localized=True and seeBall=False 

        self.rtraj = []

    def searchball(self, loc):
        if not loc.localized:
            self.turn_counterTF = 0
            # this is the case when the robot turnes in order to find the ball or to localize

            # checking if robot has already turned 3 times
            if self.turn_counterFF < 3:
                self.turn_counterFF += 1
                return {"name": "turn", "args": (math.pi / 2)}
            else:
                self.turn_counterFF = 0
                return {"name": "walk", "args": (0.5, 0)}

        else:
            # this is the case when the robot goes to the center of the field
            self.turn_counterFF = 0

            # x0,y0 - the position of the center of the field
            x0, y0 = (0.0, 0.0)

            xr, yr, yaw = loc.robot_position

            # dx,dy - vector connecting robot and center of the field
            dx = x0 - xr
            dy = y0 - yr
            dist = math.sqrt(dx ** 2 + dy ** 2)

            # checking if the robot is already at the center
            if dist < 0.07:
                if self.turn_counterTF < 3:
                    self.turn_counterTF += 1
                    return {"name": "turn", "args": (math.pi / 2)}
                else:
                    self.turn_counterTF = 0
                    return {"name": "walk", "args": (0.5, 0)}

            # ang - the angle the robot needs to turn so as to walk to the center

            ang = math.acos(dx / dist)
            if dy < 0:
                ang = -ang
            res_ang = (ang - yaw) % (2 * math.pi)
            if res_ang > math.pi:
                res_ang -= (math.pi * 2)
            return {"name": "walk", "args": (dist, res_ang)}

    def walkball(self, loc):
        # this is the case when robot is not localized but sees the ball
        self.turn_counterFF = 0
        self.turn_counterTF = 0

        # xb,yb - coords of the ball in the system of the robot
        xb = loc.ballPosSelf[0]
        yb = -loc.ballPosSelf[1]
        dist = math.sqrt(xb ** 2 + yb ** 2)

        # ang - the angle the robot needs to turn so as to walk to the ball
        ang = math.acos(xb / dist)

        # making the choice according to the distance to the ball and the angle
        if dist > 0.1:
            if yb > 0:
                return {"name": "walk", "args": (dist, -ang)}
            else:
                return {"name": "walk", "args": (dist, ang)}
        elif ang > 0.3:
            if yb > 0:
                return {"name": "turn", "args": (-ang)}
            else:
                return {"name": "turn", "args": (ang)}
        else:
            return {"name": "take_around_right", "args": (1)}

    def apply_ball_approach(self, loc, img):
        # this is the case when the robot is localized and sees the ball
        self.turn_counterFF = 0
        self.turn_counterTF = 0

        ball_approach = BallApproach()
        xr, yr, yaw = loc.robot_position
        xb, yb = loc.ball_position

        ball_approach.get_data(xr, yr, xb, yb, -yaw)

        with open('strategy/data.json') as f:
            d = json.load(f)

        ball_approach.set_constants(d)

        # traj - trajectory in the world coords
        # rtraj - trajectory in the robot coords
        # decision - action that BallApproach suggests
        traj = ball_approach.find_trajectory()
        self.traj = traj
        rtraj = ball_approach.convert_trajectory()
        self.rtraj = rtraj
        decision = ball_approach.make_decision()

        # making the choice according to the decision of BallApproach
        if decision[0] == "left kick":
            return {"name": "kick", "args": -1}
        elif decision[0] == "right kick":
            return {"name": "kick", "args": 1}
        elif decision[0] == "turn":
            return {"name": "turn", "args": decision[1]}
        elif decision[0] == "lateral step":
            return {"name": "lateral step", "args": (decision[1])}
        elif decision[0] == "take around right":
            return {"name": "take_around_right", "args": (1)}
        elif decision[0] == "take around left":
            return {"name": "take_around_left", "args": (1)}
        elif decision[0] == "walk":
            return {"name": "walk", "args": (decision[1], decision[2])}
        else:
            raise Exception("apply_ball_approach got unknown command")

    # part where the trajectory is drawn
    def draw_trajectory(self, img, model, draw=True, scale_factor=40, x0=160, y0=120,
                        field_x_sz=2.6, field_y_sz=3.6):
        if (draw is False):
            return

        # camera view
        rtraj = self.rtraj

        if (len(rtraj) == 0):
            return

        ptraj = []
        ptraj.append(model.r2pic(0.001, 0.001))

        for el in rtraj[1:]:
            ptraj.append(model.r2pic(el[0], el[1]))

        lin_num = len(ptraj) - 1

        for i in range(lin_num):
            c = int(255.0 * i / lin_num)
            color = (c, c, c)

            img.draw_line(ptraj[i][0], ptraj[i][1], ptraj[i+1][0],
                          ptraj[i+1][1], color, thickness=3)

        # bird view
        pix_x_sz = int(field_x_sz * scale_factor)
        pix_y_sz = int(field_y_sz * scale_factor)
        img.draw_rectangle(x0 - pix_x_sz // 2, y0 - pix_y_sz // 2,
                           pix_x_sz, pix_y_sz, (20, 10, 140), thickness=3)
        # goals

        traj = self.traj

        img.draw_circle(int(traj[0][1] * scale_factor) + x0, int(traj[0][0] * scale_factor) + y0,
                        5, (251, 10, 200), thickness=1, fill=True)

        for i in range(lin_num + 1):
            c = int(255.0 * i / lin_num)
            color = (c, c, c)

            if (i + 1 < len(traj)):
                img.draw_line(x0 + int(traj[i][1] * scale_factor),
                              y0 + int(traj[i][0] * scale_factor),
                              x0 + int(traj[i+1][1] * scale_factor),
                              y0 + int(traj[i+1][0] * scale_factor), color, thickness=3)

            img.draw_circle(int(traj[i][1] * scale_factor) + x0, int(traj[i][0] * scale_factor) + y0,
                            5, (190, 100, 20), thickness=1, fill=True)

    def generate_action(self, loc, img):
        # general strategy
        if loc.localized is True:
            if loc.seeBall is True:
                print('apply_ba')
                return self.apply_ball_approach(loc, img)
            else:
                print('search1')
                return self.searchball(loc)

        else:
            if loc.seeBall is True:
                print('wlkbl')
                return self.walkball(loc)

            else:
                print('search2')
                return self.searchball(loc)


# basic goalkeeper strategy - NOT USED
class gk_Strategy:
    def __init__(self):
        pass

    def gk_ball_approach(self, loc):
        if loc.seeBall:
            xb = loc.ballPosSelf[0][0]
            yb = loc.ballPosSelf[0][1]
            dist = math.sqrt(xb ** 2 + yb ** 2)
            if dist < 1:
                return {"name": "twine", "args": (1)}
            else:
                return {"name": "lateral_step", "args": yb}

    def take_def_pos(self, loc):
        pass

    def gk_generate_action(self, loc):
        if loc.localized:
            if loc.seeBall:
                return gk_ball_approach(loc)
            else:
                return take_def_pos(loc)
        else:
            if loc.seeBall:
                return gk_ball_approach(loc)
            else:
                pass
