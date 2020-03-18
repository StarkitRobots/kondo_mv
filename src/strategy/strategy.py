import sys
import math
import json
from .ball_approach import BallApproach

class Strategy:
    def __init__(self):
        self.turn_counter = 0
        pass
    
    def strat_set_conf(self, d):
        self.max_step = d["max_step"]
        self.min_step = d["min_step"]
        self.turn_step = d["turn_step"]

    def searchball(self, loc):
        if not loc.localized:
            if self.turn_counter < 3:
                self.turn_counter += 1
                
                move_chain = []
                for i in range(8):
                    move_chain.append([0.0, 0.0, -0.21])
                
            else:
                self.turn_counter = 0
                
                move_chain = []
                for i in range(20):
                    move_chain.append([0.05, 0.0, 0.0])
                
            return {"name" : "walk", "args" : move_chain}

        else:

            self.turn_counter = 0

            x0, y0 = (0, 0)
            xr0, yr0, yaw = loc.robot_position
            xr = x0 - xr0
            yr = y0 - yr0
            dist = math.sqrt(xr ** 2 + yr ** 2)
            if dist == 0:
                return 0
            ang = math.acos(xr / dist)

            if yr > 0:
                ang = -ang
            
            res_ang = ang - yaw
            res_ang %= (2 * math.pi)
            if res_ang > math.pi:
                res_ang -= (2 * math.pi)
                
            if abs(res_ang) > self.turn_step:
                num_turns = int(abs(res_ang) / self.turn_step)
                if res_ang > 0:
                    turn_ang = self.turn_step
                else:
                    turn_ang = -self.turn_step
                move_chain = []
                for i in range(num_turns):
                    move_chain.append([0.0, 0.0, turn_ang])
                
            else:
                if dist > 3 * self.max_step:
                    step_l = self.max_step
                    num_steps = int((dist - 2 * self.max_step) / step_l)
                else:
                    step_l = self.min_step
                    num_steps = int(dist / step_l)
                    
                move_chain = []
                for i in range(num_steps):
                    move_chain.append([step_l, 0.0, 0.0])


            return {"name" : "walk", "args" : move_chain}


    def walkball(self, loc):
        self.turn_counter = 0
        xb = loc.ballPosSelf[0]
        yb = -loc.ballPosSelf[1]

        dist = math.sqrt(xb ** 2 + yb ** 2)
        ang = math.acos(xb / dist)
        
        if ang > self.turn_step:
            num_turns = int(ang / self.turn_step)
            if yb < 0:
                turn_ang = -self.turn_step
            else:
                turn_ang = self.turn_step
            move_chain = []
            for i in range(num_turns):
                move_chain.append([0.0, 0.0, turn_ang])
                
        elif dist > 3 * self.max_step:
            num_steps = int((dist - 2 * self.max_step) / self.max_step)
            move_chain = []
            for i in range(num_steps):
                move_chain.append([self.max_step, 0.0, 0.0])
        
        else:
            turn_ang = self.turn_step
            chain_elem = [dist * (1 - math.cos(turn_ang)), -dist * math.sin(turn_ang), turn_ang]
            move_chain = [chain_elem]
        
        return {"name" : "walk", "args" : move_chain}

    def apply_ball_approach(self, loc):
        self.turn_counter = 0
        ba = BallApproach()
        r_pos = loc.robot_position
        b_pos = loc.ball_position

        with open('strat_conf.json') as f:
            d = json.load(f)


        ba.set_constants(d)

        dec = ba.make_decision(r_pos, b_pos)
        if dec[0] == "right kick":
            return {"name" : "kick", "args" : 1}
        elif dec[0] == "left kick":
            return {"name" : "kick", "args" : -1}
        else:
            move_chain = ba.generate_chain(r_pos, b_pos)
            return {"name" : "walk", "args" : move_chain}


    def generate_action(self, loc):
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

        return 0


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
                return self.gk_ball_approach(loc)
            else:
                return self.take_def_pos(loc)
        else:
            if loc.seeBall:
                return self.gk_ball_approach(loc)
            else:
                pass
