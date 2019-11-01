import sensor
import image
import time
import math
import json
import os
from urandom import *

def randrange(start, stop=None):
    if stop is None:
        stop = start
        start = 0
    upper = stop - start
    bits = 0
    pwr2 = 1
    while upper > pwr2:
        pwr2 <<= 1
        bits += 1
    while True:
        r = getrandbits(bits)
        if r < upper:
            break
    return r + start


def rand():
    return randrange(10000) / 10000


def gauss(mu, sigma):
    x2pi = rand() * math.pi * 2
    g2rad = math.sqrt(-2.0 * math.log(1.0 - rand()))
    z = math.cos(x2pi) * g2rad
    return mu + z * sigma


class Field:
    def __init__(self, path):
        with open(path, "r") as f:
            self.field = json.loads(f.read())
            self.w_width = self.field['main_rectangle'][0][0]
            self.w_length = self.field['main_rectangle'][0][1]


class Robot(Field):
    def __init__(self, x = 1, y = 0.5, yaw = 0):
        self.x = x          # robot's x coordinate
        self.y = y          # robot's y coordinate
        self.yaw = yaw  # robot's orientation
        self.forward_noise = 0.05   # noise of the forward movement
        self.turn_noise = 0.1      # noise of the turn
        self.sense_noise = 1   # noise of the sensing

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
        # turn, and add randomness to the turning command
        orientation = self.yaw + float(yaw) + gauss(0.0, self.turn_noise)
        orientation %= 2 * math.pi
        # move, and add randomness to the motion command
        x = self.x + x + gauss(0, self.forward_noise)
        y = self.y + y + gauss(0, self.forward_noise)
        if math.fabs(x) >= field.w_width:
            x = math.copysign(field.w_width/2.0, x)
        if math.fabs(y) >= field.w_length:
            y = math.copysign(field.w_length/2.0, y)
        self.x = x
        self.y = y
        self.yaw = orientation

    def gaussian(self, x, sigma):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return math.exp(-(x ** 2) / 2*(sigma ** 2)) / math.sqrt(2.0 * math.pi * (sigma ** 2))

    def observation_score(self, observations, landmarks): #particle weight calculation
        prob = 1.0
        matrix_means_land = []
        for landmark in landmarks:
            dists = []
            for observation in observations:
                #calc posts coords in field for every mesurement
                x_posts = self.x + observation[0]*math.sin(-self.yaw) + observation[1]*math.cos(-self.yaw)
                y_posts = self.y + observation[0]*math.cos(-self.yaw) - observation[1]*math.sin(-self.yaw)
                dist = math.sqrt((x_posts - landmark[0])**2 + (y_posts - landmark[1])**2)
                dists.append(dist)
            prob *= self.gaussian(min(dists), self.sense_noise)

        return prob

    def update_coord(self, particles):
        x = 0.0
        y = 0.0
        orientation = 0.0
        for particle in particles:
            x += particle[0].x * particle[1]
            y += particle[0].y * particle[1]
            orientation += particle[0].yaw * particle[1]
        self.x = x
        self.y = y
        self.yaw = orientation

    def return_coord(self):
        return self.x, self.y, self.orientation


class ParticleFilter():
    def __init__(self, myrobot, field,
                 n = 200, forward_noise = 0.025,
                 turn_noise = 0.1, sense_noise = 0.1, gauss_noise = 0.4):
        self.forward_noise = forward_noise
        self.turn_noise = turn_noise
        self.sense_noise = sense_noise
        self.gauss_noise = gauss_noise
        self.n = n  # number of particles
        self.myrobot = myrobot
        self.p = []
        self.yaw_noise = 0.05
        self.gen_particles()

    def custom_reset(self):
        self.p=[]
        for i in range(self.n):
            x = Robot((rand()-0.5)*field.w_width, (rand()-0.5)*field.w_length, rand()*math.pi*2)
            #x.set_noise(forward_noise, turn_noise, 0)
            self.p.append([x,0])
        self.myrobot.update_coord(self.p)

    def gen_particles(self):
        self.p = []
        for i in range(self.n):
            x_coord = self.myrobot.x + gauss(0, self.sense_noise)
            y_coord = self.myrobot.y + gauss(0, self.sense_noise)
            yaw = self.myrobot.yaw + gauss(0, self.yaw_noise)*math.pi
            yaw %= 2 * math.pi
            self.p.append([Robot(x_coord, y_coord, yaw), 0])

    def move(self, x, y, yaw):

        self.myrobot.move(x, y, yaw)
        for partic in self.p:
            partic[0].move(x, y, yaw)

    def do_n_steps(self, n_steps):
        for i in range(n_steps):
            self.step()

    def gen_n_particles_robot(self, n):
        p = []
        for i in range(n):
            x_coord = self.myrobot.x + gauss(0, self.sense_noise*3)
            y_coord = self.myrobot.y + gauss(0, self.sense_noise*3)
            yaw = self.myrobot.yaw + gauss(0, self.yaw_noise*3)*math.pi
            yaw %= 2 * math.pi
            p.append([Robot(x_coord, y_coord, yaw), 0])
        return p

    def gen_n_particles(self, n):
        tmp = []
        for i in range(n):
            x = Robot((rand()-0.5)*field.w_width, (rand()-0.5)*field.w_length, rand()*math.pi*2)
            #x.set_noise(forward_noise, turn_noise, 0)
            tmp.append([x,0])
        return tmp

    def resampling(self, measurement, landmarks):
        p_tmp = []
        w = []
        S = 0
        for i in range(self.n):
            w.append(self.p[i][0].measurement_prob(measurement))
            S += (w[i])
        for i in range(self.n):
            w[i] = w[i]/S
        index = int(rand() * self.n)
        beta = 0.0
        mw = max(w)
        new_particles = {}
        for i in range(self.n):
            beta += rand() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.n
            if index in new_particles.keys():
                new_particles[index] += 1
            else:
                new_particles[index] = 1
        for el in new_particles:
            p_tmp.append([self.p[el][0],w[el]*new_particles[el]])

        S = 0
        for i in range(len(p_tmp)):
            S += p_tmp[i][1]
        for i in range(len(p_tmp)):
            p_tmp[i][1] /= S
        self.myrobot.update_coord(p_tmp)
        new_particles = self.gen_n_particles_robot(self.n - len(p_tmp))
        p_tmp.extend(new_particles)
        self.p = p_tmp



with open("localization/landmarks.json", "r") as f:
        landmarks = json.loads(f.read())['landmarks']

def updatePF(measurement):
    field = Field("localization/parfield.json")
    robot = Robot()
    robot.set_coord(0.0, 0.0, 0.0)
    pf = ParticleFilter(robot, field, sense_noise=1.0)
    pf.resampling(measurement["red_posts"])
    return robot.return_coord()
