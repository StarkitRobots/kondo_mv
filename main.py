import sys
import sensor, image
import time, math, json
import os
import pyb

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
from vision import *

robotHeight = 0.37 #[m]
# init code
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

vision = Vision ({})
vision.load_detectors("vision/detectors_config.json")

loc=Localization(0.0, 0.5, 1.57)
strat=Strategy()
motion=Motion()
model=Model()

with open("calibration/cam_col.json") as f:
    calib=json.load(f)

# setting model parametrs
model.setParams(calib["cam_col"], robotHeight)
model.updateCameraPanTilt(0, -3.1415/6)

vision_postprocessing = Vision_postprocessing ()
motion.move_head()
t = 0
# main loop
while(True):
    clock.tick()

    curr_t = pyb.millis()
    #print (curr_t - t)
    t = curr_t
    selfData = {}
    for i in range(motion.head_state_num):

        # motion part. Head movement.
        motion.move_head()
        # vision part. Taking picture.
        img=sensor.snapshot()

        #img.save ("kekb.jpg", quality=100)

        cameraDataRaw=vision.get(img, objects_list=
        ["blue_posts", "ball", "white_posts_support"],
        drawing_list=["blue_posts", "ball", "white_posts_support"])

        cameraDataProcessed = vision_postprocessing.process (cameraDataRaw, "blue_posts", "white_posts_support")

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

    #break
    loc.update(selfData)
    print("posts number = ", len(selfData["blue_posts"]))
    print("my_pose", loc.robot_position)
    loc.update_ball(selfData)
    print(loc.ballPosSelf)

    action = strat.generate_action(loc)
    print(action)
    print(loc.pf.token)
    motion.apply(action)
    break
    time.sleep(10000)

