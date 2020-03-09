import sys
import sensor
import image
import time
import math
import json
import os
import pyb
from pyb import Pin
from pyb import LED

from common import median

sys.path.append('model')
sys.path.append('localization')
sys.path.append('motion')
sys.path.append('strategy')
sys.path.append('lowlevel')
sys.path.append('vision')
sys.path.append('imu')

from imu import IMU
from vision import Vision
from strategy import Strategy
from motion import Motion
from localization import Localization
from model import Model

# robotHeight = 0.37 #[m]
robotHeight = 0.42  # [m]

clock = time.clock()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_exposure(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False, gain_db=0)
sensor.set_auto_whitebal(False, (-6.02073, -5.11, 1.002))
sensor.set_auto_exposure(False, 2000)
vision = Vision.Vision({})
vision.load_detectors("vision/detectors_config.json")


ala = 0
side = False
ala = 1
while(ala == 0):
    if (pin9.value() == 0):   # нажатие на кнопку на голове
        ala = 1
        side = True
        print("I will attack blue goal")
        break

    if (pin3.value() == 0):
        ala = 1
        side = False
        print("I will attack yellow goal")
        break


loc = Localization(-0.7, -1.3, math.pi/2, side)
strat = Strategy.Strategy()
motion = Motion.Motion()
model = Model()
imu = IMU.IMU(0.0)
pin9 = Pin('P9', Pin.IN, Pin.PULL_UP)
pin3 = Pin('P3', Pin.IN, Pin.PULL_UP)
with open("calibration/cam_col.json") as f:
    calib = json.load(f)


# setting model parametrs
mass1 = [0, 0, 0, 0, 0, 0]
mass2 = [0, 0]
model.setParams(calib["cam_col"], robotHeight, mass1, mass2)
motion.move_head()
model.updateCameraPanTilt(0, -math.pi/6)

vision_postprocessing = Vision.Vision_postprocessing()
t = 0

# main loop
while(True):
    clock.tick()
    imu.update()
    #loc.pf.myrobot.yaw = imu.yaw/180*math.pi
    curr_t = pyb.millis()
    #print (curr_t - t)
    t = curr_t
    selfData = {}
    for i in range(12):
        # motion part. Head movement.
        a, b = motion.move_head()
        model.updateCameraPanTilt(a, b)
        # vision part. Taking picture.
        img = sensor.snapshot().lens_corr(
            strength=1.2, zoom=1.0)

        cameraDataRaw = vision.get(img, objects_list=["yellow_posts", "blue_posts", "ball"],
                                   drawing_list=["yellow_posts", "blue_posts", "ball"])

        # cameraDataRaw=vision.get(img, objects_list=["yellow_posts", "ball", "white_posts_support"],
        #                          drawing_list=["yellow_posts", "ball", "white_posts_support"])

        # vision_postprocessing.process (cameraDataRaw, "yellow_posts", "white_posts_support")
        cameraDataProcessed = cameraDataRaw
        # model part. Mapping to world coords.

        # self means in robots coords
        for observationType in cameraDataProcessed:
            if observationType not in selfData.keys():
                selfData[observationType] = []
            selfPoints = []

            # if (len (cameraDataProcessed[observationType]) > 0):
            #    print ("obser", cameraDataProcessed[observationType] [0])

            for observation in cameraDataProcessed[observationType]:
                cx = observation[5]
                cy = observation[6]
                w = observation[2]
                h = observation[3]

                selfPoints.append(model.pic2r(cx, cy + (w+h)/4))
            selfData[observationType] += selfPoints

    print("eto self yello points", #can be turned off, but now thats needs for debug
          selfData['yellow_posts'], "eto self blue points", selfData["blue_posts"])

    if len(selfData['yellow_posts']) != 0:
        general = []
        first_side = []
        second_side = []
        k = selfData['yellow_posts'][0]
        for pep in selfData['yellow_posts']:
            if math.fabs(math.atan(pep[0]/pep[1]) - math.atan(k[0]/k[1])) < 0.3:
                first_side.append(list(pep))
            else:
                second_side.append(list(pep))
        if len(first_side) != 1:
            first_side = median(first_side)
        else:
            first_side = first_side[0]

        if second_side != 0 and len(second_side) != 1:
            second_side = median(second_side)
        elif len(second_side) == 1:
            second_side = second_side[0]

        general.append(first_side)
        if len(second_side) != 0:
            general.append(second_side)
        selfData['yellow_posts'] = general
    print("eto self yello points", #can be turned off, but now thats needs for debug
          selfData['yellow_posts'], "eto self blue points", selfData["blue_posts"]) 

    loc.update(selfData)
    loc.update_ball(selfData)
    loc.localized = True  #can be turned off, but now thats needs for debug
    # print(loc.ballPosSelf)

    action = strat.generate_action(loc, img)
    print(action)#can be turned off, but now thats needs for debug

    strat.draw_trajectory(img, model)

    # print(loc.pf.token)

    odometry_results = motion.apply(action)
    print("odometry = ", odometry_results['shift_x'],
          odometry_results['shift_y'], odometry_results['shift_yaw']*180/math.pi) #can be turned off, but now thats needs for debug
    if odometry_results is not None:
        loc.pf.particles_move(odometry_results)