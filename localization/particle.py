import math
import sys

sys.path.append('localization')
sys.path.append('../tools')

from random import Random
from robot import Robot

#TODO : mb we should to calc landmark pos in self coord instead of calc 
#       every observation in field coord. May work a little faster.
#       
#       mb move prob to Particle class variable

class Particle(Robot):
    def observation_score(self, observations, landmarks, sense_noise):
        # particle weight calculation
        prob = 1.0
        for color_landmarks in observations:
            if (color_landmarks not in landmarks):
                continue
            for landmark in landmarks[color_landmarks]:
                dists = []
                if observations[color_landmarks]:
                    for observation in observations[color_landmarks]:
                        # calc posts coords in field for every mesurement
                        x_posts, y_posts = get_field_coords_from_self(self, observation)
                        dist = math.sqrt(
                            (x_posts - landmark[0])**2 + (y_posts - landmark[1])**2)
                        dists.append(dist)
                if (dists != []):
                    prob *= Random.gaussian(min(dists), sense_noise)
        return prob


    def get_field_coords_from_self(self, observation):
        x_field= self.x + observation[0] * math.cos(self.yaw) -
                          observation[1] * math.sin(self.yaw)
        y_field = self.y + observation[0] * math.sin(self.yaw) + 
                           observation[1] * math.cos(self.yaw)
        return (x_field, y_field)
        
    def get_self_coords_from_field(self, observation):
        #implement func
        return (x_self, y_self)

    def lines_dist(self, l1, l2):
        return dist_particle2line(l1, particle) - dist_particle2line(l2, particle)

    def dist_particle2line(self, l):
        #implement func 
        return distance

    def calc_lines_score(self, lines, landmarks):
        '''
        line = (ro, theta)
        '''
        prob = 1
        dists = []
        for landmark_line_field in landmarks["lines"]:
            landmark_line_self = self.get_self_coords_from_field(landmark_line_field)
            for line in lines:
                dists.append(self.lines_dist(line, landmark_line_self))
        if (dists != []):
            prob *= Random.gaussian(min(dists), sense_noise)

