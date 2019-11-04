import sys
import sensor, image
import time, math, json
import os
import pyb

from model.model import Model
sys.path.append('localization')
from localization import Localization
sys.path.append('motion')
from motion import Motion
sys.path.append('strategy')
from strategy import Strategy
sys.path.append('lowlevel')
#from lowlevel import *
sys.path.append('vision')
from vision import Vision, Detector, ColoredObjectDetector, BallDetector, SurroundedObjectDetector


robotHeight = 0.37 #[m]
# init code
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

field = (40, 85, -55, 25, -15, 55)

vision = Vision({#"ball": ColoredObjectDetector((30, 80, 0, 40, -10, 20)),
    #"blue_posts": ColoredObjectDetector((20, 55, 40, 80, 30, 70)),

    #(self, obj_th_, surr_th_, sector_rad_ = 50, wind_sz_ = 3,
    #pixel_th_ = 300, area_th_ = 300, merge_ = False,
    #points_num_ = 10, min_ang_ = 0, max_ang_ = 2, objects_num_ = 1):

    #"blue_posts": SurroundedObjectDetector((0, 20, -10, 30, -45, 10),
    #                                   (40, 60, -60, -10, 0, 45),
    "blue_posts": SurroundedObjectDetector((5, 22, 5, 55, -53, -10),
                                          field,
                                       sector_rad_ = 30,
                                           min_ang_ = 0,
                                           max_ang_ = 3.14,
                                           points_num_ = 7,
                                           objects_num_ = 2),
   "ball": SurroundedObjectDetector((30, 80, 40, 85, 10, 60),
                                         field,
                                      sector_rad_ = 30,
                                          min_ang_ = 0,
                                          max_ang_ = 6.1,
                                          points_num_ = 10,
                                          objects_num_ = 1)})

    #"yellow_posts": ColoredObjectDetector((20, 55, 40, 80, 30, 70))})

loc=Localization()
strat=Strategy()
motion=Motion()
model=Model()

with open("calibration/cam_col.json") as f:
    calib=json.load(f)

# setting model parametrs
model.setParams(calib["cam_col"], robotHeight)
model.updateCameraPanTilt(0, -3.1415/6)

t = 0

# main loop
ite = 1
for i in range(16):
#while(True):
    clock.tick()

    curr_t = pyb.millis()
    #print (curr_t - t)
    t = curr_t

    for b in range(1):

        # motion part. Head movement.
        #motion.move_head()

        # vision part. Taking picture.
        img=sensor.snapshot()

        #img.save ("kekb.jpg", quality=100)

        cameraData=vision.get(
            img, objects_list=["blue_posts", "ball"],#, "yellow_posts"],
            drawing_list=["blue_posts", "ball"])#, "yellow_posts"])

        # model part. Mapping to world coords.

        # self means in robots coords
        selfData={}
        for observationType in cameraData:
            selfPoints = []
            for observation in cameraData[observationType]:
                selfPoints.append(
                    model.pic2r(observation[0] + observation[2]/2,
                    (observation[1] + observation[3])))
            selfData[observationType] = selfPoints
        print(selfData)

    #break
    if (i  == 8):
        time.sleep(7000)
        #a, b, c = input()
        #a, b, c = map(float, input().split())
        #if ite < 6:
           # loc.move(0.0, -0.0, 0.0)
        #if ite > 5 and ite < 15:
            #loc.move(-0.55,0.0,0.0)
        #if ite > 5 and ite < 15:
        loc.move(-0.7,0.0,0.0)
        print("thats motion",loc.pf.myrobot.return_coord())


    loc.update(selfData)
    #action = strat.generate_action(loc)

    #motion.apply(action)
print(loc.pf.token)
print('end')
