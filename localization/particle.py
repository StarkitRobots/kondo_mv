import sys

sys.path.append('localization')
from robot import Robot
sys.path.append('localization/tools')
from random import Random
import math

class Particle(Robot):
      #def __init__(self, x = 0, y = 0, yaw = 0):
       # self.x = x          # robot's x coordinate
        #self.y = y          # robot's y coordinate
        #self.yaw = yaw  # robot's orientation
        #self.forward_noise = 0.05   # noise of the forward movement
        #self.turn_noise = 0.1      # noise of the turn
        #self.sense_noise = 1.7   # noise of the sensing

    def observation_score(self, observations, landmarks, sense_noise):
        #particle weight calculation
        prob = 1.0
        for color_landmarks in observations:
            if (color_landmarks not in landmarks):
                continue

            for landmark in landmarks[color_landmarks]:
                dists = []
                if observations[color_landmarks]:
                    for observation in observations[color_landmarks]:
                    #calc posts coords in field for every mesurement
                        x_posts = self.x + observation[0]*math.cos(-self.yaw) + observation[1]*math.sin(-self.yaw)
                        y_posts = self.y - observation[0]*math.sin(-self.yaw) + observation[1]*math.cos(-self.yaw)
                        dist = math.sqrt((x_posts - landmark[0])**2 + (y_posts - landmark[1])**2)
                        dists.append(dist)
                if (dists!=[]):
                    prob *= Random.gaussian(min(dists), sense_noise)
        return prob


def set_noise(self, new_forward_noise, new_turn_noise, new_sense_noise):
        self.forward_noise = float(new_forward_noise)
        self.turn_noise = float(new_turn_noise)
        self.sense_noise = float(new_sense_noise)
