import sys
import sensor, image
import time, math, json
import os
import pyb
from pyb import Pin
from pyb import LED

sys.path.append('model')
from model import Model
sys.path.append('localization')
from localization import Localization
sys.path.append('motion')
from motion import Motion
sys.path.append('strategy')
from strategy import Strategy
sys.path.append('lowlevel')
#from lowlevel import *
sys.path.append('vision')
from vision import Vision
sys.path.append('imu')
from imu import IMU

#robotHeight = 0.37 #[m]
robotHeight = 0.42 #[m]
# init code
#sensor.reset()
#sensor.set_pixformat(sensor.RGB565)
#sensor.set_framesize(sensor.QVGA)
#sensor.skip_frames(time=2000)
#sensor.set_auto_gain(False)  # must be turned off for color tracking
#sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False, 9.1)  # must be turned off for color tracking
#sensor.set_auto_whitebal(False, (0.1343897, -6.02073, 2.716595))
sensor.set_auto_exposure(False, 6500) # must be turned off for color tracking

vision = Vision.Vision ({})
vision.load_detectors("vision/detectors_config.json")

ala = 0
side = False
ala = 1
while(ala==0):
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


loc=Localization(0.0, 0.0, 0.0, side)
strat=Strategy.Strategy()
motion=Motion.Motion()
model=Model()
imu = IMU.IMU(0)
pin9 = Pin('P9', Pin.IN, Pin.PULL_UP)
pin3 = Pin('P3', Pin.IN, Pin.PULL_UP)
with open("calibration/cam_col.json") as f:
    calib=json.load(f)




# setting model parametrs
mass1 = [0,0,0,0,0,0]
mass2 = [0,0]
model.setParams(calib["cam_col"], robotHeight,mass1, mass2)
#motion.move_head()
#model.updateCameraPanTilt(0, -3.1415/6)

vision_postprocessing = Vision.Vision_postprocessing ()
t = 0

# main loop
while(True):
    clock.tick()
    imu.update()
    loc.pf.myrobot.yaw = imu.yaw/180*math.pi
    curr_t = pyb.millis()
    #print (curr_t - t)
    t = curr_t
    selfData = {}
    for i in range(12 ):
        # motion part. Head movement.
        a, b = motion.move_head()
        model.updateCameraPanTilt(a,b)
        # vision part. Taking picture.
        img=sensor.snapshot()

        #img.save ("kekb.jpg", quality=100)

        cameraDataRaw=vision.get(img, objects_list=["yellow_posts", "ball"],
                                      drawing_list=["yellow_posts", "ball"])

        #cameraDataRaw=vision.get(img, objects_list=["yellow_posts", "ball", "white_posts_support"],
        #                          drawing_list=["yellow_posts", "ball", "white_posts_support"])

        cameraDataProcessed = cameraDataRaw#vision_postprocessing.process (cameraDataRaw, "yellow_posts", "white_posts_support")
        # model part. Mapping to world coords.

        # self means in robots coords
        for observationType in cameraDataProcessed:
            if observationType not in selfData.keys():
                selfData[observationType] = []
            selfPoints = []
            for observation in cameraDataProcessed[observationType]:
                selfPoints.append(
                    model.pic2r(observation[0] + observation[2]/2,
                    observation[1] + observation[3]))
            selfData[observationType]+=selfPoints

        #print("keys = ", selfData.keys())

    print ("eto self points", selfData['yellow_posts'])

    #break
    print("eto loc baall", selfData['ball'] )

    loc.update(selfData)
    #print("posts number = ", len(selfData["yellow_posts"]))
    #
    #print("my_pose", loc.robot_position)
    loc.update_ball(selfData)
    #loc.ball_position = (1.2, 0.0)
    #loc.robot_position = (0.0, 0.0, 0.0)
    loc.localized = True
    #loc.seeBall = True
    #print(loc.ballPosSelf)
    #loc.robot_position = (0.75, 0, 0)
    #loc.ball_position = (1.25, 0)

    action = strat.generate_action(loc, img)
    print(action)

    strat.draw_trajectory (img, model)

    #print(loc.pf.token)

    odometry_results = motion.apply(action)
    #if odometry_results is not None:
        #loc.pf.move(odometry_results)
