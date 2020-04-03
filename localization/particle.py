import math
import sys

sys.path.append('localization')
sys.path.append('../tools')

from Random import Random
from robot import Robot

#TODO : mb we should to calc landmark pos in self coord instead of calc 
#       every observation in field coord. May work a little faster.
#       
#       mb move prob to Particle class variable

class Particle(Robot):
    def observation_score(self, observations, landmarks, gauss_noise):
        # particle weight calculation
        prob = 1.0
        for color_landmarks in observations:
            if (color_landmarks not in landmarks):
                continue
            if (color_landmarks == 'lines'):
                continue
            for landmark in landmarks[color_landmarks]:
                dists = []
                if observations[color_landmarks]:
                    for observation in observations[color_landmarks]:
                        # calc posts coords in field for every mesurement
                        x_posts = self.x + \
                            observation[0] * math.cos(self.yaw) - \
                            observation[1] * math.sin(self.yaw)
                        y_posts = self.y + \
                            observation[0] * math.sin(self.yaw) + \
                            observation[1] * math.cos(self.yaw)
                        dist = math.sqrt(
                            (x_posts - landmark[0])**2 + (y_posts - landmark[1])**2)
                        dists.append(dist)
                    prob *= Random.gaussian(min(dists), gauss_noise)
        return prob


    def get_field_coords_from_self(self, observation):
        x_field= self.x + observation[0] * math.cos(self.yaw) - \
                          observation[1] * math.sin(self.yaw)
        y_field = self.y + observation[0] * math.sin(self.yaw) + \
                           observation[1] * math.cos(self.yaw)
        return (x_field, y_field)
        
    def get_self_coords_from_field(self, observation):
        #implement func
        return (x_self, y_self)

    #def lines_dist(self, l1, l2):
     #   return dist_particle2line(l1, particle) - dist_particle2line(l2, particle)

    #def dist_particle2line(self, l):
        #implement func 
     #   return distance

    def calc_lines_score(self, lines, landmarks, gauss_noise):
        '''
        line = (ro, theta)
        '''
        prob = 1
        dists = []
        if lines != []:
            for line in lines:
                for landmark_line in landmarks:
                    for coord in landmark_line:
                #landmark_line_self = self.get_self_coords_from_field(landmark_line_field)
                        yaw = (self.yaw + line[1])%(2*math.pi)
                        if landmark_line == 'x':
                            dist = math.fabs(coord - (self.x + line[0]*math.cos(yaw)))
                        else:
                            dist = math.fabs(coord - (self.y + line[0]*math.sin(yaw)))
                        dists.append(dist)
            if (dists != []):
                prob *= Random.gaussian(min(dists), gauss_noise)
        return prob

