import sensor
import image
import time
import math
import json
import os
import sys
import uio
sys.path.append('localization/tools/')
from random import Random
sys.path.append('localization')
import Robot
from particle import Particle




#TODO make Field.py
class Field:
    def __init__(self, path):
        with open(path, "r") as f:
            self.field = json.loads(f.read())
            self.w_width = self.field['main_rectangle'][0][0]
            self.w_length = self.field['main_rectangle'][0][1]

#TODO make Robot.py
#TODO remove inheritance from Field
#robot: coord, move, remove all with noise


#TODO all constants to JSON
#TODO
#TODO delete all unused functions
class ParticleFilter():
    def __init__(self, myrobot, field, landmarks,
                 n = 100, forward_noise = 0.025,
                 turn_noise = 0.1, sense_noise = 0.4, gauss_noise = 0.4,
                 consistency = 0.0, dist_threshold = 0.5, goodObsGain = 0.1,
                 badObsCost = 0.1, stepCost = 0.1 ):
        self.token = str(Random.random()*10000)
        self.forward_noise = forward_noise
        self.turn_noise = turn_noise
        self.sense_noise = sense_noise
        self.gauss_noise = gauss_noise
        self.logs = open("localization/logs/logs"+ self.token + '.txt',"w")
        self.n = n  # number of particles
        self.count = 0
        self.myrobot = myrobot
        self.p = []
        self.yaw_noise = 0.05
        self.gen_particles()
        self.logs.close()
        self.landmarks = landmarks
        #logs = uio.open('localization/logs/logs.txt',"w")
        #sys.stdout = self.logs

        self.consistency = consistency
        self.goodObsGain = goodObsGain
        self.badObsCost = badObsCost
        self.stepCost = stepCost
        self.dist_threshold = dist_threshold

    def uniform_reset(self):
        self.p=[]
        for i in range(self.n):
            x = Robot((random()-0.5)*field.w_width, (random()-0.5)*field.w_length, random()*math.pi*2)
            #x.set_noise(forward_noise, turn_noise, 0)
            self.p.append([x,0])
        self.myrobot.update_coord(self.p)

    def update_consistency(self, observations):
        #prob = self.myrobot.observation_score(observations)
        stepConsistency = 0
        for color_landmarks in observations:
            if (color_landmarks not in self.landmarks):
                continue

            for landmark in self.landmarks[color_landmarks]:
                dists = []
                if len(observations[color_landmarks]) != 0:
                    for observation in observations[color_landmarks]:
                #calc posts coords in field for every mesurement
                        x_posts = (self.myrobot.x + observation[0]*math.sin(-self.myrobot.yaw)
                                   + observation[1]*math.cos(-self.myrobot.yaw))
                        y_posts = (self.myrobot.y + observation[0]*math.cos(-self.myrobot.yaw)
                                   - observation[1]*math.sin(-self.myrobot.yaw))
                        dist = math.sqrt((x_posts - landmark[0])**2 + (y_posts - landmark[1])**2)
                        dists.append(dist)
                    if min(dists) < self.dist_threshold:
                        stepConsistency += self.goodObsGain
                    else:
                        stepConsistency -= self.badObsCost
                else:
                    stepConsistency -= self.stepCost
        #print(stepConsistency)
        self.consistency += stepConsistency

    def gen_particles(self):
        print('initial,step ', self.count, file=self.logs)
        print("$$", file=self.logs)
        print("position ", self.myrobot.x, ' ',
              self.myrobot.y, ' ', self.myrobot.yaw, '|', file=self.logs)
        self.p = []
        for i in range(self.n):
            x_coord = self.myrobot.x + Random.gauss(0, self.sense_noise)
            y_coord = self.myrobot.y + Random.gauss(0, self.sense_noise)
            yaw = self.myrobot.yaw + Random.gauss(0, self.yaw_noise)*math.pi
            yaw %= 2 * math.pi
            self.p.append([Particle(x_coord, y_coord, yaw), 0])
            print(x_coord, ' ', y_coord, ' ', yaw, file=self.logs)
        #print('|', file = self.logs)
        self.count += 1
    #TODO rename
    def move(self, coord):
        self.logs = open('localization/logs/logs'+self.token+'.txt',"a")
        self.myrobot.move(coord['shift_x'], coord['shift_y'], coord['shift_yaw'])
        print('|moving,step ', self.count, file=self.logs)
        print('$$', file=self.logs)
        print("position ", self.myrobot.x, ' ',
              self.myrobot.y, ' ', self.myrobot.yaw, '|', file=self.logs)
        # now we simulate a robot motion for each of
        # these particles
        for partic in self.p:
            partic[0].move(coord['shift_x'], coord['shift_y'], coord['shift_yaw'])
            print(partic[0].x, ' ',
              partic[0].y, ' ', partic[0].yaw, file=self.logs)
        #print('|', file = self.logs)
        self.count += 1
        self.logs.close()


    def gen_n_particles_robot(self, n):
        p = []
        for i in range(n):
            x_coord = self.myrobot.x + Random.gauss(0, self.sense_noise*3)
            y_coord = self.myrobot.y + Random.gauss(0, self.sense_noise*3)
            yaw = self.myrobot.yaw + Random.gauss(0, self.yaw_noise)*math.pi
            yaw %= 2 * math.pi
            p.append([Robot(x_coord, y_coord, yaw), 0])
        return p

    def gen_n_particles(self, n):
        tmp = []
        for i in range(n):
            x = Robot((random()-0.5)*field.w_width, (random()-0.5)*field.w_length, random()*math.pi*2)
            #x.set_noise(forward_noise, turn_noise, 0)
            tmp.append([x,0])
        return tmp

    def observation_to_predict(self, observations):
        predicts = []
        for color_landmarks in observations:
            if (color_landmarks not in self.landmarks):
                continue

            for landmark in self.landmarks[color_landmarks]:
                if len(observations[color_landmarks]) != 0:
                    for obs in observations[color_landmarks]:
                        x_posts = self.myrobot.x - obs[0]*math.sin(-self.myrobot.yaw) + obs[1]*math.cos(-self.myrobot.yaw)
                        y_posts = self.myrobot.y + obs[0]*math.cos(-self.myrobot.yaw) - obs[1]*math.sin(-self.myrobot.yaw)
                        predicts.append([x_posts, y_posts])
        return predicts

    #TODO refactor
    def resampling(self, observations):
        self.logs =open('localization/logs/logs'+self.token+'.txt',"a")
        print('|resempling,step ', self.count, file=self.logs)
        print('$', self.observation_to_predict(observations), '$', file=self.logs)
        p_tmp = []
        w = []
        S = 0
        for i in range(self.n):
            w.append(self.p[i][0].observation_score(observations, self.landmarks))
            S += (w[i])
        for i in range(self.n):
            w[i] = w[i]/S
            #S += w[i]
        index = int(random() * self.n)
        beta = 0.0
        mw = max(w)
        #print(mw)
        new_particles = {}
        for i in range(self.n):
            beta += random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.n
            if index in new_particles.keys():
                new_particles[index] += 1
            else:
                new_particles[index] = 1
            #p_tmp.append([self.p[index][0],w[index]])
        for el in new_particles:
            p_tmp.append([self.p[el][0],w[el]*new_particles[el]])
        S = 0
        for i in range(len(p_tmp)):
            S += p_tmp[i][1]
        for i in range(len(p_tmp)):
            p_tmp[i][1] /= S
        self.update_coord(p_tmp)
        self.update_consistency(observations)
        print("position ", self.myrobot.x, ' ',
              self.myrobot.y, ' ', self.myrobot.yaw, '|', file=self.logs)
        new_particles = self.gen_n_particles_robot(self.n - len(p_tmp))
        p_tmp.extend(new_particles)
        self.p = p_tmp
        for particle in p_tmp:
            print(particle[0].x, ' ',
              particle[0].y, ' ', particle[0].yaw, file=self.logs)
        #print('|', file = self.logs)
        self.count += 1
        self.update_consistency(observations)
        self.logs.close()

    def custom_reset(self, x, y, yaw):
        self.myrobot.x = x
        self.myrobot.y = y
        self.myrobot.yaw = yaw
        self.p = gen_n_particles_robot(self.n)

    def fall_reset(self, observations):
        self.update_consistency(observations)
        self.custom_reset(self.myrobot.x + Random.gauss(0, self.sense_noise),
                         self.myrobot.y + Random.gauss(0, self.sense_noise),
                         self.myrobot.y  + Random.gauss(0, self.sense_noise))
        self.resampling(observations)

    def update_coord(self, particles):
        x = 0.0
        y = 0.0
        orientation = 0.0
        for particle in particles:
            x += particle[0].x * particle[1]
            y += particle[0].y * particle[1]
            orientation += particle[0].yaw * particle[1]
        self.myrobot.x = x
        self.myrobot.y = y
        self.myrobot.yaw = orientation


def updatePF(pf, measurement):
    pf.resampling(measurement)
    #print(pf.myrobot.return_coord())
    return pf.myrobot.return_coord()
