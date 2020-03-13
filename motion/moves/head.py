import json
import sys
sys.path.append('motion/moves')
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

    def start(self):
        data = []
        #state_num = len(self.head_motion_states['enter'])
        if self.frames_to_process == []:    
            self.enabled = True
            #self.state = ['enter', (self.state[1] + 1) % state_num]
            for state in self.head_motion_states['enter'].values():
                self.servos['head_yaw'] = -state['head_yaw']
                self.servos['head_pitch'] = state['head_pitch']
                data.append(self.servos)
            #print("pitch " + str(self.kondo.getSinglePos(1)[1] + " yaw " + self.kondo.getSinglePos(1)[1])
            self.frames_to_process = data
        return data

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
            self.frames_to_process = data
        return data
            
    def stop(self):
        data = []
        #state_num = len(self.head_motion_states['exit'])
        if self.enabled:
            #self.state = ['enter', (self.state[1] + 1) % state_num]
            for state in self.head_motion_states['exit'].values():
                self.servos['head_yaw'] = -state['head_yaw']
                self.servos['head_pitch'] = state['head_pitch']
                data.append(self.servos)
            #print("pitch " + str(self.kondo.getSinglePos(1)[1] + " yaw " + self.kondo.getSinglePos(1)[1])
            self.frames_to_process = data
            self.enabled = False
        return data

if __name__ == "__main__":
    head = Head()
    print(Head.__bases__)
    print(isinstance(head, Move))
    print(type(head))