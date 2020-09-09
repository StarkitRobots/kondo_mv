import json
from .move import Move

class Kick(Move):
    def __init__(self):
        super().__init__()
        # kick init
        with open("motion/moves/kick.json", "r") as f:
            self.kick_states = json.loads(f.read())
        self.enabled = True
        self.name = 'kick'
        self.side = 1

    def enter(self):
        pass

    def tick(self):
        data = []
        if self.enabled:    
            if self.side == -1:
                side_temp = 'left'
            elif self.side == 1:
                side_temp = 'right'
            else:
                pass

            for state in self.kick_states['tick'].values():
                servos = {}
                for servo in state:
                    servos[side_temp + '_' + servo] = state[servo]
                data.append(servos)
   
            self.frames_to_process += data
        return data

    def exit(self):
        pass