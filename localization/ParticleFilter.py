
import time
import math
import json
import os
import uio
from .field import Field
from .tools import Random
from .robot import Robot
from .particle import Particle

class ParticleFilter():
    def __init__(self, myrobot, field, landmarks,n = 100):

        self.n = n
        self.myrobot = myrobot
        self.landmarks = landmarks
        self.field = field
        self.count = 0
        self.p = []

        with open('localization/pf_constants.json', 'r') as constants:
            constants = json.load(constants)

        self.forward_noise = constants['noise']['forward_noise']
        self.turn_noise = constants['noise']['turn_noise']
        self.sense_noise = constants['noise']['sense_noise']
        self.gauss_noise = constants['noise']['gauss_noise']
        self.yaw_noise = constants['noise']['yaw_noise']

        self.number_of_res = constants['consistency']['number_of_res']
        self.consistency = constants['consistency']['consistency']
        self.goodObsGain = constants['consistency']['goodObsGain']
        self.badObsCost = constants['consistency']['badObsCost']
        self.stepCost = constants['consistency']['stepCost']
        self.dist_threshold = constants['consistency']['dist_threshold']
        self.con_threshold = constants['consistency']['con_threshold']
        self.spec_threshold = constants['consistency']['spec_threshold']

        self.token = str(Random().random()*10000)
        self.logs = open("localization/logs/logs"+ self.token + '.txt',"w")
        self.gen_particles()
        self.logs.close()
        print('scence noise', self.sense_noise)

    def return_coord(self):
        return self.myrobot.x, self.myrobot.y, self.myrobot.yaw


    def gen_particles(self):
        print('initial,step ', self.count, file=self.logs)
        print("$$", file=self.logs)
        print("position ", self.myrobot.x, ' ',
              self.myrobot.y, ' ', self.myrobot.yaw, '|', file=self.logs)
        self.p = []
        for i in range(self.n):
            x_coord = self.myrobot.x + Random().gauss(0, self.sense_noise)
            y_coord = self.myrobot.y + Random().gauss(0, self.sense_noise)
            yaw = self.myrobot.yaw + Random().gauss(0, self.yaw_noise)*math.pi
            #print('yaw'+str(i),yaw*180/math.pi)
            if yaw < 0:
                yaw = 2*math.pi + yaw
            #print('yaw'+str(i),yaw*180/math.pi)
            if yaw > 2*math.pi:
                yaw%= (2 * math.pi*180/math.pi)
            #print('yaw'+str(i),yaw*180/math.pi)
            self.p.append([Particle(x_coord, y_coord, yaw), 0])
            print(x_coord, ' ', y_coord, ' ', yaw, file=self.logs)
        #print('|', file = self.logs)
        self.count += 1

    def gen_n_particles_robot(self, n):
        p = []
        for i in range(n):
            x_coord = self.myrobot.x + Random().gauss(0, self.sense_noise*3)
            y_coord = self.myrobot.y + Random().gauss(0, self.sense_noise*3)
            yaw = self.myrobot.yaw + Random().gauss(0, self.yaw_noise)*math.pi
            if yaw < 0:
                yaw = 2*math.pi + yaw
            if yaw > 2*math.pi:
                yaw%= (2 * math.pi)
            p.append([Particle(x_coord, y_coord, yaw), 0])
        return p

    def uniform_reset(self):
        self.p=[]
        for i in range(self.n):
            x = Robot((Random().random()-0.5)*\
            self.field.w_width, (Random().random()-0.5)*\
            self.field.w_length, Random().random()*math.pi*2)
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
                        x_posts = (self.myrobot.x + observation[0]*math.cos(self.myrobot.yaw)
                        - observation[1]*math.sin(self.myrobot.yaw))
                        y_posts = (self.myrobot.y + observation[0]*math.sin(self.myrobot.yaw)
                        + observation[1]*math.cos(self.myrobot.yaw))

                        dist = math.sqrt((x_posts - landmark[0])**2 + (y_posts - landmark[1])**2)
                        dists.append(dist)
                        #print('dist, len =', dist, len(dists))
                    if min(dists) < self.dist_threshold:
                        stepConsistency += self.goodObsGain
                        #print('good step', stepConsistency)
                    else:
                        stepConsistency -= self.badObsCost
                        #print('bad step', stepConsistency)
                else:
                    stepConsistency -= self.stepCost
        #print('step cons', stepConsistency)
        self.consistency += stepConsistency
        if math.fabs(self.consistency) > self.spec_threshold:
            self.consistency = math.copysign(self.spec_threshold, self.consistency)
        print('consistency', self.consistency)


    def particles_move(self, coord):
        self.logs = open('localization/logs/logs'+self.token+'.txt',"a")
        self.myrobot.move(coord['shift_x'], coord['shift_y'], coord['shift_yaw'])
        print('eto coord after mooving', self.return_coord(), self.myrobot.yaw*180/math.pi)
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
            x_coord = self.myrobot.x + Random().gauss(0, self.sense_noise*3)
            y_coord = self.myrobot.y + Random().gauss(0, self.sense_noise*3)
            yaw = self.myrobot.yaw + Random().gauss(0, self.yaw_noise)*math.pi
            if yaw < 0:
                yaw = 2*math.pi + yaw
            if yaw > 2*math.pi:
                yaw%= (2 * math.pi)
            p.append([Particle(x_coord, y_coord, yaw), 0])
        return p

    def gen_n_particles(self, n):
        tmp = []
        for i in range(n):
            x = Robot((Random().random()-0.5)*self.field.w_width, (Random().random()-0.5)*self.field.w_length, Random().random()*math.pi*2)
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
                        y_posts = self.myrobot.x + obs[0]*math.sin(-self.myrobot.yaw) + obs[1]*math.cos(-self.myrobot.yaw)
                        x_posts = self.myrobot.y + obs[0]*math.cos(-self.myrobot.yaw) - obs[1]*math.sin(-self.myrobot.yaw)
                        predicts.append([x_posts, y_posts])
        return predicts


    def resampling_wheel(self, weights, p_tmp):
        new_particles = {}
        index = int(Random().random() * self.n)
        beta = 0.0
        mw = max(weights)
        #print(mw)
        for i in range(self.n):
            beta += Random().random() * 2.0 * mw
            while beta > weights[index]:
                beta -= weights[index]
                index = (index + 1) % self.n
            if index in new_particles.keys():
                new_particles[index] += 1
            else:
                new_particles[index] = 1
            #p_tmp.append([self.p[index][0],w[index]])
        for el in new_particles:
            p_tmp.append([self.p[el][0],weights[el]*new_particles[el]])
        return p_tmp



    def resampling(self, observations):
        self.logs =open('localization/logs/logs'+self.token+'.txt',"a")
        print('|resempling,step ', self.count, file=self.logs)
        print('$', self.observation_to_predict(observations), '$', file=self.logs)
        p_tmp = []
        w = []
        S = 0
        for i in range(self.n):
            w.append(self.p[i][0].observation_score(observations, self.landmarks, self.sense_noise))
            S += (w[i])
        for i in range(self.n):
            w[i] = w[i]/S
            #S += w[i]
        self.resampling_wheel(w, p_tmp)
        S = 0
        for i in range(len(p_tmp)):
            S += p_tmp[i][1]
        for i in range(len(p_tmp)):
            p_tmp[i][1] /= S
            #if p_tmp[i][1] > 0.02:
                #print("x y yaw ", p_tmp[i][0].x, p_tmp[i][0].y, p_tmp[i][0].yaw*180/math.pi)
        self.update_coord(p_tmp)
        self.update_consistency(observations)
    #def rationing(weights, sum):
        #for i in range(len(weights))
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
        self.p = self.gen_n_particles_robot(self.n)


    # ------------------------------
    # need to add to handle the fall
    # ------------------------------
    def fall_reset(self, observations):
        self.update_consistency(observations)
        self.custom_reset(self.myrobot.x + Random().gauss(0, self.sense_noise),
                         self.myrobot.y + Random().gauss(0, self.sense_noise),
                         self.myrobot.y  + Random().gauss(0, self.sense_noise))
        self.resampling(observations)


    def update_coord(self, particles):
        x = 0.0
        y = 0.0
        orientation = 0.0
        adding = 0.0
        if (self.myrobot.yaw < math.pi/2) or (self.myrobot.yaw > math.pi*3/2):
            adding = math.pi*2
        for particle in particles:
            x += particle[0].x * particle[1]
            y += particle[0].y * particle[1]
            if (particle[0].yaw < math.pi):
                culc_yaw = particle[0].yaw + adding
            else:
                culc_yaw = particle[0].yaw
            orientation += culc_yaw * particle[1]
        self.myrobot.x = x
        self.myrobot.y = y
        self.myrobot.yaw = orientation % (2*math.pi)

    def return_coord(self):
        return self.myrobot.x, self.myrobot.y, self.myrobot.yaw



def updatePF( pf, measurement):
    k = pf.number_of_res
    if pf.consistency < pf.con_threshold:
        k+=1
    for i in range(3):
        pf.resampling(measurement)
    print('eto coord', pf.return_coord(), pf.myrobot.yaw*180/math.pi)
    return pf.return_coord()
