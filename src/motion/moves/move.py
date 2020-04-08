import json

class Move(object):
    def __init__(self, config_file):
        self.servos = {}
        self.name = ''
        self.enabled = False
        self.frames_to_process = []
        self.states = json.loads(config_file)

    def get_frame(self):
        if self.frames_to_process != []:
            return self.frames_to_process.pop(0)
        else:
            return []
    
    def start(self): 
        data = []
        if self.frames_to_process == []:    
            self.enabled = True
            # input your code here to manage servos
            data.append(self.servos)
            self.frames_to_process = data
        return data

    def tick(self):
        data = []
        if self.enabled:    
            # input your code here to manage servos
            data.append(self.servos)
            self.frames_to_process += data
        return data

    def stop(self):
        data = []  
        # input your code here to manage servos
        data.append(self.servos)
        self.frames_to_process += data
        self.enabled = False
        return data

    def read_state_from_json(self, state, state_num):
        return self.states[state][state_num]