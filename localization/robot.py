import math

class Robot():
    def __init__(self, x = 0, y = 0, yaw = 0):
        self.x = x          # robot's x coordinate
        self.y = y          # robot's y coordinate
        self.yaw = yaw  # robot's orientation
        self.forward_noise = 0.05   # noise of the forward movement
        self.turn_noise = 0.1      # noise of the turn
        self.sense_noise = 1.7   # noise of the sensing

    def set_coord(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.yaw = float(new_orientation)

    def set_noise(self, new_forward_noise, new_turn_noise, new_sense_noise):
        self.forward_noise = float(new_forward_noise)
        self.turn_noise = float(new_turn_noise)
        self.sense_noise = float(new_sense_noise)

    def sense(self, landmarks):
        z = []
        for i in range(len(landmarks)):
            dist = math.sqrt((self.x - landmarks[i][0]) ** 2
                             + (self.y - landmarks[i][1]) ** 2)
            dist += gauss(0.0, self.sense_noise)
            z.append(dist)
        return z

    def move(self, x, y, yaw):
        # turn, and add randomomness to the turning command
        orientation = self.yaw + float(yaw) + gauss(0.0, self.turn_noise)
        orientation %= 2 * math.pi
        # move, and add randomomness to the motion command
        x = self.x + x + gauss(0, self.forward_noise)
        y = self.y + y + gauss(0, self.forward_noise)
        #if math.fabs(x) >= field.w_width:
            #x = math.copysign(field.w_width/2.0, x)
        #if math.fabs(y) >= field.w_length:
           # y = math.copysign(field.w_length/2.0, y)
        self.x = x
        self.y = y
        self.yaw = orientation


    def observation_to_predict(self, observations, landmarks):
        predicts = []
        for color_landmarks in landmarks:
            if (color_landmarks not in landmarks):
                continue

            for landmark in landmarks[color_landmarks]:
                x_posts = self.x - observation[0]*math.sin(-self.yaw) + observation[1]*math.cos(-self.yaw)
                y_posts = self.y + observation[0]*math.cos(-self.yaw) - observation[1]*math.sin(-self.yaw)
                predicts.append([x_posts, y_posts])
        return predicts
    #TODO to pf 
   

