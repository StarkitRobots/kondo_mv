import json
from move import Move

class Head(Move):
    def __init__(self):
        super().__init__()
        # head init
        with open("motion/moves/head.json", "r") as f:
            self.head_motion_states = json.loads(f.read())
        self.enabled = True
        self.pan = 0   
        self.tilt = 0
        self.state = ['', -1]

    def enter(self):
        data = []
        #state_num = len(self.head_motion_states['enter'])
        if self.enabled:
            #self.state = ['enter', (self.state[1] + 1) % state_num]
            for state in self.head_motion_states['enter'].values():
                self.servos['head_yaw'] = -state['head_yaw']
                self.servos['head_pitch'] = state['head_pitch']
                data.append(self.servos)
            #print("pitch " + str(self.kondo.getSinglePos(1)[1] + " yaw " + self.kondo.getSinglePos(1)[1])
            return data
        else:
            return []

    # discrete head motion that understands its position and does next step:
    def tick(self):
        data = []
        #state_num = len(self.head_motion_states['tick'])
        if self.enabled:
            #self.state = ['enter', (self.state[1] + 1) % state_num]
            for state in self.head_motion_states['tick'].values():
                servos = {}
                servos['head_yaw'] = -state['head_yaw']
                servos['head_pitch'] = state['head_pitch']
                data.append(servos)
                
            #print("pitch " + str(self.kondo.getSinglePos(1)[1] + " yaw " + self.kondo.getSinglePos(1)[1])
            return data
        else:
            return []
            
    def exit(self):
        data = []
        #state_num = len(self.head_motion_states['exit'])
        if self.enabled:
            #self.state = ['enter', (self.state[1] + 1) % state_num]
            for state in self.head_motion_states['exit'].values():
                self.servos['head_yaw'] = -state['head_yaw']
                self.servos['head_pitch'] = state['head_pitch']
                data.append(self.servos)
            #print("pitch " + str(self.kondo.getSinglePos(1)[1] + " yaw " + self.kondo.getSinglePos(1)[1])
            return data
        else:
            return []