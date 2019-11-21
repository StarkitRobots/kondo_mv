import sys
import math
import json

from ball_approach import ball_approach

class Strategy:
    def __init__(self):
        self.turn_counter = 0
        pass

    def searchball(self, loc):
        if not loc.localized:
            # this is the case when the robot turnes in order to find the ball or to localize

            # checking if robot has already turned 3 times
            if self.turn_counter < 3:
                self.turn_counter += 1
                return {"name" : "turn", "args" : (math.pi / 2)}
            else:
                self.turn_counter = 0
                return {"name" : "walk", "args" : (1, 0)}

        else:
            # this is the case when the robot goes to the center of the field
            self.turn_counter = 0

            # x0,y0 - the position of the center of the field 
            x0, y0 = (0, 0)

            xr, yr, yaw = loc.robot_position

            # dx,dy - vector connecting robot and center of the field
            dx = x0 - xr
            dy = y0 - yr
            dist = math.sqrt(dx ** 2 + dy ** 2)

            # checking if the robot is already at the center
            if dist == 0:
                raise Exception('going to the center while already at the center')

            # ang - the angle the robot needs to turn so as to walk to the center
            ang = math.acos(dx / dist)
            if dy > 0:
                ang = -ang

            return {"name" : "walk", "args" : (dist, ang - yaw)}


    def walkball(self, loc):
        # this is the case when robot is not localized but sees the ball
        self.turn_counter = 0

        # xb,yb - coords of the ball in the system of the robot
        xb = loc.ballPosSelf[0]
        yb = -loc.ballPosSelf[1]
        dist = math.sqrt(xb ** 2 + yb ** 2)

        # ang - the angle the robot needs to turn so as to walk to the ball
        ang = math.acos(xb / dist)

        # making the choice according to the distance to the ball and the angle
        if dist > 0.15:
            if yb > 0:
                return {"name" : "walk", "args" : (dist, -ang)}
            else:
                return {"name" : "walk", "args" : (dist, ang)}
        elif ang > 0.3:
            if yb > 0:
                return {"name" : "turn", "args" : (-ang)}
            else:
                return {"name" : "turn", "args" : (ang)}
        else:
            return {"name" : "take_around_right", "args" : (1)}

    def apply_ball_approach(self, loc):
        # this is the case when the robot is localized and sees the ball
        self.turn_counter = 0
        ball_approach = BallApproach()
        xr, yr, yaw = loc.robot_position
        xb, yb = loc.ball_position

        ball_approach.get_data(xr, yr, xb, yb, -yaw)

        with open('data.json') as f:
            d = json.load(f)

        ball_approach.set_constants(list(d.values()))

        # traj - trajectory in the world coords
        # rtraj - trajectory in the robot coords
        # decision - action that BallApproach suggests
        traj = ball_approach.find_trajectory()
        rtraj = ball_approach.convert_trajectory()
        decision = ball_approach.make_decision()
        dist = math.sqrt(rtraj[1][0] ** 2 + rtraj[1][1] ** 2)

        # making the choice according to the decision of BallApproach and the distance to the ball
        if dist > d["min_dist"]:
            ang = math.acos(rtraj[1][0] / dist)
            if rtraj[1][1] > 0:
                return {"name" : "walk", "args" : (dist, -ang)}
            else:
                return {"name" : "walk", "args" : (dist, ang)}
        elif decision == "turn left" or decision == "turn right":
            ang = math.acos(rtraj[1][0] / dist)
            if rtraj[1][1] > 0:
                return {"name" : "turn", "args" : (-ang)}
            else:
                return {"name" : "turn", "args" : (ang)}
        elif decision == "step left" or decision == "step right":
            return {"name" : "lateral_step", "args" : (rtraj[1][1])}
        elif decision == "strike":
            return {"name" : "kick", "args" : (1)}


    def generate_action(self, loc):
        # general strategy
        if loc.localized == True:
            if loc.seeBall == True:
                return self.apply_ball_approach(loc)
            else:
                return self.searchball(loc)

        else:
            if loc.seeBall == True:
                return self.walkball(loc)

            else:
                return self.searchball(loc)


class gk_Strategy:
    def __init__(self):
        pass

    def gk_ball_approach(self, loc):
        if loc.seeBall:
            xb = loc.ballPosSelf[0][0]
            yb = loc.ballPosSelf[0][1]
            dist = math.sqrt(xb ** 2 + yb ** 2)
            if dist < 1:
                return {"name" : "twine", "args" : (1)}
            else:
                return {"name" : "lateral_step", "args" : yb}


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
