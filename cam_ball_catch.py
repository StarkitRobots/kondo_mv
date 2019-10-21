import math

class cam_ball_catch:
    def get_data(self, rxb, ryb, h):
        self.rxb = rxb
        self.ryb = ryb
        self.h = h
    
    def find_angles(self):
    
        h = self.h
        rxb = self.rxb
        ryb = self.ryb
        
        if rxb == 0:
            return None
        
        polarb = -1 * math.atan(h / math.sqrt(rxb ** 2 + ryb ** 2))
        azimb = math.atan(ryb / rxb)
        
        return polarb, azimb
