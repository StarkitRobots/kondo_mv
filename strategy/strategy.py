import sys
import math
import json
import random

from ball_approach import ball_approach
sys.path.append("..")
from localization.localization import Localization


class Strategy:
    def __init__(self):
        self.turn_counter = 0
        pass

    def searchball(self, loc):
        if self.turn_counter < 3:
            return {"turn", (math.pi / 2)}
        else:
            self.turn_counter = 0
            if loc.localized:
                xr, yr, yaw = loc.robot_position
                dist = math.sqrt(xr ** 2 + yr ** 2)
                if dist == 0:
                    return "no idea"
                ang = math.acos(xr / dist)
                if yr < 0:
                    ang = -ang
                dang = yaw - ang
                if dang > 0:
                    rang = math.pi - dang
                elif dang < 0:
                    rang = dang - math.pi
                return {"walk", (dist, rang)}
            else:
                dist = 100 * random.random()
                ang = 2 * math.pi * (random.random() - 0.5)
                return {"walk", (dist, ang)}





    def generate_action(self, loc):
        if loc.localized == True:
            if loc.seeBall == True:
                ba = ball_approach()
                xr, yr, yaw = loc.robot_position
                xb, yb = loc.ball_position

                ba.get_data(xr, yr, xb, yb, yaw)

                with open('data.json') as f:
                    d = json.load(f)
                    
                dv = list(d.values())
                ba.set_constants(list(d.values()))

                traj = ba.find_trajectory()
                print(traj)
                rtraj = ba.convert_trajectory()
                dec = ba.make_decision()
                dist = math.sqrt(rtraj[1][0] ** 2 + rtraj[1][1] ** 2)

                if dist > dv[1]:
                    ang = math.acos(rtraj[1][0] / dist)
                    if rtraj[1][1] > 0:
                        return {"walk", (dist, ang)}
                    else:
                        return {"walk", (dist, -1 * ang)}
                elif dec == "turn left" or dec == "turn right":
                    ang = math.acos(rtraj[1][0] / dist)
                    if rtraj[1][1] > 0:
                        return {"turn", (ang)}
                    else:
                        return {"turn", (-1 * ang)}
                elif dec == "step left" or dec == "step right":
                    if rtraj[1][1] > 0:
                        return {"side move", (rtraj[1][1])}
                    else:
                        return {"side move", (-1 * rtraj[1][1])}
                elif dec == "strike":
                    return {"strike", (1)}
                
            else:
                return searchball(self, loc)
        else:
            if loc.seeBall == True:
                xb, yb = loc.ballPosSelf

                dist = math.sqrt(xb ** 2 + yb ** 2)
                ang = math.acos(xb / dist)
                if dist > 1:
                    if yb > 0:
                        return {"walk", (dist, ang)}
                    else:
                        return {"walk", (dist, -1 * ang)}
                elif ang < 1:
                    if yb > 0:
                        return {"turn", (ang)}
                    else:
                        return {"turn", (-1 * ang)}
                else:
                    return {"take around right", (1)}


            else:
                return searchball(self, loc)


        return 0