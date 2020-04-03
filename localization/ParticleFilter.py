import math
import json
import os
import sys
import time

sys.path.append('localization/tools/')
sys.path.append('localization')

from Random import Random
from particle import Particle
from robot import Robot
import random #временно

class ParticleFilter():
    def __init__(self, robot, field, landmarks, n=100):

        self.number_of_particles = n
        self.robot = robot
        self.landmarks = landmarks
        self.count = 0
        self.particles = []

        with open('localization/pf_constants.json', 'r') as constants:
            constants = json.load(constants)

        self.forward_noise = constants['noise']['forward_noise']
        self.turn_noise = constants['noise']['turn_noise']
        self.sense_noise = constants['noise']['sense_noise']
        self.gauss_noise = constants['noise']['gauss_noise']
        self.yaw_noise = constants['noise']['yaw_noise']
        self.line_gauss_noise = constants['noise']['line_gauss_noise']

        self.number_of_res = constants['consistency']['number_of_res']
        self.consistency = constants['consistency']['consistency']
        self.goodObsGain = constants['consistency']['goodObsGain']
        self.badObsCost = constants['consistency']['badObsCost']
        self.stepCost = constants['consistency']['stepCost']
        self.dist_threshold = constants['consistency']['dist_threshold']
        self.con_threshold = constants['consistency']['con_threshold']
        self.spec_threshold = constants['consistency']['spec_threshold']

        self.token = str(random.random() * 10000)
        self.logs = open("localization/logs/logs" + '.txt', "w")
        self.gen_particles()
        self.logs.close()

    def return_coord(self):
        return self.robot.x, self.robot.y, self.robot.yaw

    def gen_particles(self):
        print('initial,step ', self.count, file=self.logs)
        print("$$", file=self.logs)
        print("position ", self.robot.x, ' ',
              self.robot.y, ' ', self.robot.yaw, '|', file=self.logs)
        self.particles = []
        for i in range(self.number_of_particles):
            x_coord = self.robot.x + random.gauss(0, self.sense_noise)
            y_coord = self.robot.y + random.gauss(0, self.sense_noise)
            yaw = self.robot.yaw + random.gauss(0, self.yaw_noise)*math.pi
            if yaw < 0:
                yaw = 2*math.pi + yaw
            if yaw > 2*math.pi:
                yaw %= (2 * math.pi)
            self.particles.append([Particle(x_coord, y_coord, yaw), 0])
            print(x_coord, ' ', y_coord, ' ', yaw, file=self.logs)
        self.count += 1

    def gen_n_particles_robot(self, n):
        particles = []
        for i in range(n):
            x_coord = self.robot.x + random.gauss(0, self.sense_noise*3)
            y_coord = self.robot.y + random.gauss(0, self.sense_noise*3)
            yaw = self.robot.yaw + random.gauss(0, self.yaw_noise)*math.pi
            if yaw < 0:
                yaw = 2 * math.pi + yaw
            if yaw > 2 * math.pi:
                yaw %= (2 * math.pi)
            particles.append([Particle(x_coord, y_coord, yaw), 0])
        return particles

    def uniform_reset(self):
        self.particles = []
        #
        #need to redo
        #
        #for i in range(self.number_of_particles):
        #    x = Robot((random.random()-0.5) * field.w_width, (random.random()-0.5)
        #              * field.w_length, random.random()*math.pi * 2)
        #    self.particles.append([x, 0])
        self.robot.update_coord(self.particles)

    def update_consistency(self, observations):
        stepConsistency = 0
        for color_landmarks in observations:
            if (color_landmarks not in self.landmarks):
                continue
            if len(observations[color_landmarks]) != 0:
                for observation in observations[color_landmarks]:
                    dists = []
                    for landmark in self.landmarks[color_landmarks]:
                        # calc posts coords in field for every mesurement
                        x_posts = (self.robot.x + observation[0] * math.cos(self.robot.yaw)
                                   - observation[1] * math.sin(self.robot.yaw))
                        y_posts = (self.robot.y + observation[0] * math.sin(self.robot.yaw)
                                   + observation[1] * math.cos(self.robot.yaw))
                        dist = math.sqrt(
                            (x_posts - landmark[0])**2 + (y_posts - landmark[1])**2)
                        dists.append(dist)
                    if min(dists) < self.dist_threshold:
                        stepConsistency += self.goodObsGain
                    else:
                        stepConsistency -= self.badObsCost
                        #print('bad step', stepConsistency)
                # else:
                    #stepConsistency -= self.stepCost
        #print('step cons', stepConsistency)
        self.consistency += stepConsistency
        if math.fabs(self.consistency) > self.spec_threshold:
            self.consistency = math.copysign(
                self.spec_threshold, self.consistency)
        print('consistency', self.consistency)

    def particles_move(self, coord):
        self.logs = open('localization/logs/logs'+self.token+'.txt', "a")
        self.robot.move(coord['shift_x'],
                          coord['shift_y'], coord['shift_yaw'])
        print('eto coord after mooving', self.return_coord(),
              self.robot.yaw*180/math.pi)
        print('|moving,step ', self.count, file=self.logs)
        print('$$', file=self.logs)
        print("position ", self.robot.x, ' ',
              self.robot.y, ' ', self.robot.yaw, '|', file=self.logs)
        # now we simulate a robot motion for each of
        # these particles
        for particle in self.particles:
            particle[0].move(coord['shift_x'], coord['shift_y'],
                           coord['shift_yaw'])
            print(particle[0].x, ' ',
                  particle[0].y, ' ', particle[0].yaw, file=self.logs)
        #print('|', file = self.logs)
        self.count += 1
        self.logs.close()

    def gen_n_particles(self, n):
        tmp = []
        #
        #need to redo
        #
        #for i in range(n):
        #    x = Robot((random.random()-0.5) * field.w_width, (random()-0.5)
        #              * field.w_length, random.random() * math.pi*2)
         #   tmp.append([x, 0])
        return tmp

    def observation_to_predict(self, observations):
        predicts = []
        for color_landmarks in observations:
            if (color_landmarks not in self.landmarks):
                continue
            for landmark in self.landmarks[color_landmarks]:
                if len(observations[color_landmarks]) != 0:
                    for obs in observations[color_landmarks]:
                        y_posts = self.robot.x + \
                            obs[0] * math.sin(-self.robot.yaw) + \
                            obs[1] * math.cos(-self.robot.yaw)
                        x_posts = self.robot.y + \
                            obs[0] * math.cos(-self.robot.yaw) - \
                            obs[1] * math.sin(-self.robot.yaw)
                        predicts.append([x_posts, y_posts])
        return predicts

    def resampling_wheel(self, weights, res_particles):
        new_particles = {}
        index = int(random.random() * self.number_of_particles)
        beta = 0.0
        mw = max(weights)
        for i in range(self.number_of_particles):
            beta += random.random() * 2.0 * mw
            while beta > weights[index]:
                beta -= weights[index]
                index = (index + 1) % self.number_of_particles
            if index in new_particles.keys():
                new_particles[index] += 1
            else:
                new_particles[index] = 1
        for el in new_particles:
            res_particles.append([self.particles[el][0], weights[el]*new_particles[el]])
        return res_particles

    def resampling(self, observations):
        self.logs = open('localization/logs/logs'+self.token+'.txt', "a")
        print('|resempling,step ', self.count, file=self.logs)
        print('$', self.observation_to_predict(
            observations), '$', file=self.logs)
        res_particles = []
        w = []
        S = 0
        for i in range(self.number_of_particles):
            w.append(self.particles[i][0].observation_score(
                observations, self.landmarks, self.gauss_noise)*self.particles[i][0].calc_lines_score(
                observations['lines'], self.landmarks['lines'], self.line_gauss_noise))
            S += (w[i])
        for i in range(self.number_of_particles):
            w[i] = w[i]/S
        self.resampling_wheel(w, res_particles)
        S = 0
        for i in range(len(res_particles)):
            S += res_particles[i][1]
        for i in range(len(res_particles)):
            res_particles[i][1] /= S
        self.update_coord(res_particles)
        self.update_consistency(observations)
        print("position ", self.robot.x, ' ',
              self.robot.y, ' ', self.robot.yaw, '|', file=self.logs)
        new_particles = self.gen_n_particles_robot(self.number_of_particles - len(res_particles))

        res_particles.extend(new_particles)
        self.particles = res_particles
        for particle in res_particles:
            print(particle[0].x, ' ',
                  particle[0].y, ' ', particle[0].yaw, file=self.logs)
        #print('|', file = self.logs)
        self.count += 1
        self.update_consistency(observations)
        self.logs.close()

    def custom_reset(self, x, y, yaw):
        self.robot.x = x
        self.robot.y = y
        self.robot.yaw = yaw
        self.p = self.gen_n_particles_robot(self.number_of_particles)

    # ------------------------------
    # need to add to handle the fall
    # ------------------------------

    def fall_reset(self, observations):
        self.custom_reset(self.robot.x + random.gauss(0, self.sense_noise),
                          self.robot.y + random.gauss(0, self.sense_noise),
                          self.robot.yaw + random.gauss(0, self.yaw_noise))
        self.resampling(observations)
        self.update_consistency(observations)

    def update_coord(self, particles):
        x = 0.0
        y = 0.0
        orientation = 0.0
        if (self.robot.yaw < math.pi/2) or (self.robot.yaw > math.pi * 3/2):
            yaw_add = math.pi * 2
        else:
            yaw_add = 0.0
        for particle in particles:
            x += particle[0].x * particle[1]
            y += particle[0].y * particle[1]
            if (particle[0].yaw < math.pi):
                culc_yaw = particle[0].yaw + yaw_add
            else:
                culc_yaw = particle[0].yaw
            orientation += culc_yaw * particle[1]
        self.robot.x = x
        self.robot.y = y
        self.robot.yaw = orientation % (2 * math.pi)



def updatePF(pf, measurement):
    for i in range(pf.number_of_res):
        pf.resampling(measurement)
    print('eto coord', pf.return_coord(), pf.robot.yaw*180/math.pi)
    return pf.return_coord()
