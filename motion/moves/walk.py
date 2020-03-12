import sys
import json 
import math
sys.path.append('..')
from walk_engine import WalkEngine
from move import Move


class Walk(Move):
    def __init__(self, use_engine=False, model=None):
        super().__init__()
        self.enabled = False
        # Use walk engine or not
        self._use_engine = use_engine
        if self._use_engine:
            if model is not None:
                self.engine = WalkEngine(model)
            else:
                raise Exception('No model')
        self.walk_step = 0
        self.lateral_step = 0
        self.walk_turn = 0
        self.cycle = 0 
        self.cycles_num = 0
        # load config file
        with open("motion/moves/walk.json", "r") as f:
            self._walk_config = json.loads(f.read())
        self.max_blind_distance = self._walk_config['max_blind_distance']
        self.max_lateral = self._walk_config['max_lateral']
        self.max_turn = self._walk_config['max_turn'] * math.pi / 180.

    def enter(self):
        self.enabled = True
        if self._use_engine:
            return self.engine.enter()
        else: 
            return {} 

    def tick(self):
        if self.enabled:
            if self._use_engine:
                self.engine.update(self.walk_step, 
                                    self.lateral_step, 
                                    self.walk_turn, 
                                    self.cycle, 
                                    self.cycles_num)
                return self.engine.tick()
            else:
                step_num = int(self.lateral_step / self.max_lateral)
                if step_num > 0:
                    return {'motion':'Soccer_Small_Step_Left', 'args':{'c1': step_num, 'u1': 0}}
                else:
                    return {'motion':'Soccer_Small_Step_Right', 'args':{'c1': abs(step_num), 'u1': 0}}

                if abs(self.walk_turn) > self.max_turn:
                    # hardcode piece of Kefir code (don't do it like that next time)
                    def _get_turn_params(target):
                        uo = -150
                        co = 1

                        err = 360

                        if (abs (target) > 360):
                            #print ("idi naher")
                            print('Wrong parameter')

                        neg = False

                        if target < 0:
                            neg = True
                            target *= -1

                        for u in range(-150, 150, 10):
                            for c in range(1, target // 15 + 1):
                                curr = -u * 0.12 * c
                                curr_err = abs (target - curr)

                                if curr_err < err:
                                    err = curr_err
                                    uo = u
                                    co = c

                        if (neg == True):
                            uo *= -1
                        return co, uo
                    c1, u1 = _get_turn_params(self.walk_turn)
                    return {'motion':'Soccer_Turn', 'args':{'c1': c1, 'u1': u1}}

                if self.walk_step < self.max_blind_distance:
                    step_num = 1
                else:
                    step_num = 3
                return {'motion': 'Soccer_WALK_FF', 'args':{'c1': step_num, 'u1': 1}}
        else:
            return {}    

    def exit(self):
        self.enabled = False
        if self._use_engine:
            return self.engine.exit()
        else:
            return {}