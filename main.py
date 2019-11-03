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


robotHeight = 0.41 #[m]
# init code
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()


vision = Vision({#"ball": ColoredObjectDetector((30, 80, 0, 40, -10, 20)),
    #"blue_posts": ColoredObjectDetector((20, 55, 40, 80, 30, 70)),

    #(self, obj_th_, surr_th_, sector_rad_ = 50, wind_sz_ = 3,
    #pixel_th_ = 300, area_th_ = 300, merge_ = False,
    #points_num_ = 10, min_ang_ = 0, max_ang_ = 2, objects_num_ = 1):

    "blue_posts": SurroundedObjectDetector((0, 20, -10, 30, -45, 10),
                                           (40, 60, -60, -10, 0, 45),
                                           sector_rad_ = 30,
                                           min_ang_ = 0,
                                           max_ang_ = 3.14,
                                           points_num_ = 7,
                                           objects_num_ = 2)})

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
for i in range(10):
#while(True):
    clock.tick()


    for i in range(5):

        # motion part. Head movement.
        #motion.move_head()

        # vision part. Taking picture.
        img=sensor.snapshot()
        #img = rgb_to_lab (img)

        cameraData=vision.get(
            img, objects_list=["blue_posts"],#, "yellow_posts"],
            drawing_list=["blue_posts"])#, "yellow_posts"])

        # model part. Mapping to world coords.

        # self means in robots coords
        selfData={}
        for observationType in cameraData:
            selfPoints = []
            for observation in cameraData[observationType]:
                selfPoints.append(
                    model.pic2r(observation[0] - observation[2]/2,
                    observation[1] - observation[3]))
            selfData[observationType] = selfPoints


    loc.update(selfData)
    action = strat.generate_action(loc)

    motion.apply(action)

loc.end_of_loc()
print("end")
