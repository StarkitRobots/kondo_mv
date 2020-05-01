"""
Class entiny for ball, robot and particles. This class contains fields with coordinates and angle of object and 
"""

import math
import sys

sys.path.append('localization')
sys.path.append('../tools/')

from random_tools import Random


class Entity():
    def __init__(self, x = 0, y = 0, yaw = 0):
        self.x = x          # Object's x coordinate
        self.y = y          # Object's y coordinate
        self.yaw = yaw      # Object's angle

    def set_coord(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.yaw = float(new_orientation)

    def move(self, x, y, yaw):
        # turn, and add randomomness to the turning command
        orientation = self.yaw + float(yaw)
        if orientation < 0:
            orientation += (math.pi * 2)
        orientation %= (2 * math.pi)
        self.x += x*math.cos(self.yaw)
        self.y += x*math.sin(self.yaw)
        self.yaw = orientation
    
    def local_to_global_coord(self, observation):
        
        x_field= self.x + observation[0] * math.cos(self.yaw) - \
                          observation[1] * math.sin(self.yaw)
        y_field = self.y + observation[0] * math.sin(self.yaw) + \
                           observation[1] * math.cos(self.yaw)
        return (x_field, y_field)

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
                        x_posts, y_posts = self.local_to_global_coord(observation)
                        dist = math.sqrt(
                            (x_posts - landmark[0])**2 + (y_posts - landmark[1])**2)
                        dists.append(dist)
                    prob *= Random.gaussian(min(dists), gauss_noise)
        return prob

    def calc_lines_score(self, lines, landmarks, gauss_noise):
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
