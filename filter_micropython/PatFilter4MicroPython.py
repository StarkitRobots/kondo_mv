import sensor, image, time, math
from urandom import *

p_tmp = []
sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time=2000)  # Wait for settings take effect.
clock = time.clock()  # Create a clock object to track the FPS.


# Note: OpenMV Cam runs about half as fast when connected
# to the IDE. The FPS should increase once disconnected.

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


import json


class Field:
    def __init__(self, field):
        self.field = figures
        self.w_width = self.field['main_rectangle'][0][0]
        self.w_length = self.field['main_rectangle'][0][1]
    
    #def __init(self, path):
    #    with open(path, "r") as f:
    #        self.field = json.loads(f.read())
    #    self.w_width = self.field['main_rectangle'][0][0]
    #    self.w_length = self.field['main_rectangle'][0][1]


class Robot(Field):
    def __init__(self, x=1, y=0.5, yaw=0):

        self.x = x  # robot's x coordinate
        self.y = y  # robot's y coordinate
        self.orientation = yaw  # robot's orientation

        self.x_pred = 0.0
        self.y_pred = 0.0
        self.forward_noise = 0.05  # noise of the forward movement
        self.turn_noise = 0.1  # noise of the turn
        self.sense_noise = 0.3  # noise of the sensing

    def set_coord(self, new_x, new_y, new_orientation):
        # if new_orientation < 0 or new_orientation >= 2 * pi:
        #   raise ValueError('Orientation must be in [0..2pi]')

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

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
        # print("z = ", z)
        return z

    def move(self, turn, forward):
        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + gauss(0.0, self.turn_noise)
        orientation %= 2 * math.pi

        # move, and add randomness to the motion command
        dist = float(forward) + gauss(0.0, self.forward_noise)
        x = self.x + (math.cos(orientation) * dist)
        y = self.y + (math.sin(orientation) * dist)

        # set particle
        res = Robot()
        res.set_coord(x, y, orientation)
        # res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)

        return res

    def gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return math.exp(-((mu - x) ** 2) / (sigma ** 2)) / math.sqrt(2.0 * math.pi * (sigma ** 2))

    def measurement_prob(self, measurement):
        prob = 1.0
        for i in range(len(landmarks)):
            dist = math.sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            # dist = landmarks[i][0]
            # print("distVSmeas = ", dist - measurement[i])
            # print(self.gaussian(dist, self.sense_noise, measurement[i]))
            prob *= self.gaussian(dist, self.sense_noise, measurement[i][0])
            # print("prob ", prob)
        return prob

    def update_coord(self, particles):
        x = 0.0
        y = 0.0
        for i in range(len(particles)):
            x += particles[i][0].x * particles[i][1]
            y += particles[i][0].y * particles[i][1]
            # x += particles[i][0]*math.cos(particles[i][1]) * particles[i][1]
            # y += particles[i][0]*math.sin(particles[i][1]) * particles[i][1]
        # self.x_pred = float(x/len(particles))
        # self.y_pred = float(y/len(particles))

        self.x = x
        self.y = y

    def return_coord(self):
        return self.x, self.y


class ParticleFilter():
    def __init__(self, myrobot, field,
                 n=100, forward_noise=0.025,
                 turn_noise=0.1, sense_noise=0.3):
        self.forward_noise = forward_noise
        self.turn_noise = turn_noise
        self.sense_noise = sense_noise
        self.n = n  # number of particles
        self.myrobot = myrobot
        self.p = []

        for i in range(self.n):
            x = Robot((rand() - 0.5) * field.w_width, (rand() - 0.5) * field.w_length, rand() * math.pi * 2)
            # x.set_noise(forward_noise, turn_noise, 0)
            self.p.append([x, 0])

    def step(self):

        self.myrobot = self.myrobot.move(0, 0.02)
        z = self.myrobot.sense()

        # now we simulate a robot motion for each of
        # these particles
        p_tmp = []
        p = self.p
        for i in range(self.n):
            p_tmp.append(p[i][0].move(0, 0.02))
        self.p = p_tmp

        return p_tmp

    def do_n_steps(self, n_steps):
        for i in range(n_steps):
            self.step()

    def resampling(self, measurement):
        p_tmp = []
        w = []
        S = 0
        for i in range(self.n):
            z = self.myrobot.sense((landmarks))
            # z = get_measurements(landmarks)
            w.append(self.p[i][0].measurement_prob(measurement))
            S += (w[i])
        print("px =  ", p[i][0].x)
        # print("w ", w )
        for i in range(self.n):
            w[i] = w[i] / S
            S += w[i]
        index = int(rand() * self.n)
        beta = 0.0
        mw = max(w)
        # pd.Series(w).hist(bins=20)
        # print(mw)
        for i in range(self.n):
            beta += rand() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.n
            p_tmp.append([self.p[index][0], w[index]])
        # print(p_tmp)
        S = 0
        for i in range(len(p_tmp)):
            S += p_tmp[i][1]
        for i in range(len(p_tmp)):
            p_tmp[i][1] /= S

        self.p = p_tmp
        self.myrobot.update_coord(p_tmp)
        return w, p_tmp


figures = {
    "circles": [
        [0, 0, 1]
    ],
    "lines": [
        [[-4.5, 4.5], [0, 0]]
    ],
    "points": [
        [0, 0]
    ],
    "main_rectangle": [
        [1, 1]
    ],
    "rectangles": [
        [[-1, -4.5], 2, 1], [[-1, 3.5], 2, 1]
    ]
}
#path = "untitled.json"
#measurement = [[3.8809736656992513, 2.44685437739309], [2.8061306051321995, 1.9513027039072615]]
landmarks = [[0.4, 0], [0.6, 0]]

def updatePF(measurement):
    field = Field(figures)
    robot = Robot()
    robot.set_coord(0.0, 0.0, 0.0)
    pf = ParticleFilter(robot, field, sense_noise=1.0)
    pf.resampling(measurement)
    return robot.return_coord()