import sys
import math
import json

from ball_approach import ball_approach
sys.path.append("../localization")
from localization.localization import Localization


class Strategy:
    def __init__(self):
        self.turn_counter = 0
        pass

    def searchball(self, loc):
        if self.turn_counter < 3:
            self.turn_counter += 1
            return {"turn" : (math.pi / 2)}
        else:
            self.turn_counter = 0
            if loc.localized:
                x0, y0 = (180, 130)
                xr0, yr0, yaw = loc.robot_position
                xr = x0 - xr0
                yr = y0 - yr0
                dist = math.sqrt(xr ** 2 + yr ** 2)
                if dist == 0:
                    return "no idea"
                ang = math.acos(xr / dist)
                
                if yr < 0:
                    ang = -ang
                
                return {"walk" : (dist, ang - yaw)}
            else:
                return {"walk" : (1, 0)}
            

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
                rtraj = ba.convert_trajectory()
                dec = ba.make_decision()
                dist = math.sqrt(rtraj[1][0] ** 2 + rtraj[1][1] ** 2)

                if dist > dv[1]:
                    ang = math.acos(rtraj[1][0] / dist)
                    if rtraj[1][1] > 0:
                        return {"walk" : (dist, ang)}
                    else:
                        return {"walk", (dist, -1 * ang)}
                elif dec == "turn left" or dec == "turn right":
                    ang = math.acos(rtraj[1][0] / dist)
                    if rtraj[1][1] > 0:
                        return {"turn" : (ang)}
                    else:
                        return {"turn", (-1 * ang)}
                elif dec == "step left" or dec == "step right":
                    if rtraj[1][1] > 0:
                        return {"side move" : (rtraj[1][1])}
                    else:
                        return {"side move" : (-1 * rtraj[1][1])}
                elif dec == "strike":
                    return {"strike" : (1)}
                
            else:
                return self.searchball(loc)
        else:
            if loc.seeBall == True:
                xb, yb = loc.ballPosSelf

                dist = math.sqrt(xb ** 2 + yb ** 2)
                ang = math.acos(xb / dist)
                if dist > 1:
                    if yb > 0:
                        return {"walk" : (dist, ang)}
                    else:
                        return {"walk" : (dist, -1 * ang)}
                elif ang < 1:
                    if yb > 0:
                        return {"turn" : (ang)}
                    else:
                        return {"turn" : (-1 * ang)}
                else:
                    return {"take around right" : (1)}


            else:
                return self.searchball(loc)


        return 0