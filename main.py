import sys
import sensor, image
import time, math, json
import os
import pyb

sys.path.append('/model')
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
vision.load_detectors ("vision/detectors_config.json")

loc=Localization(-0.3, 0.9, 1.57)
strat=Strategy()
motion=Motion()
model=Model()

with open("calibration/cam_col.json") as f:
    calib=json.load(f)

# setting model parametrs
model.setParams(calib["cam_col"], robotHeight)
model.updateCameraPanTilt(0, -3.1415/6)

#vision_postprocessing = Vision_postprocessing ("blue_posts", ["left_blue_post", "right_blue_post"])

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
        #motion.move_head()
<<<<<<< HEAD
        model.updateCameraPanTilt(motion.head_yaw * math.pi / 180., motion.head_pitch * math.pi / 180.)
=======

>>>>>>> 280988b1d4785684fb35834ac83790085a67c75d
        # vision part. Taking picture.
        img=sensor.snapshot()

        #img.save ("kekb.jpg", quality=100)

        cameraDataRaw=vision.get(
            img, objects_list=["blue_posts", "ball", "white_posts_support"],#, "yellow_posts"],
            drawing_list=["blue_posts", "ball", "white_posts_support"])#, "yellow_posts"])

        posts_num   = len (cameraDataRaw ["blue_posts"])
        support_num = len (cameraDataRaw ["white_posts_support"])

        left_post  = []
        right_post = []

        if (posts_num == 2):
            post1 = cameraDataRaw ["blue_posts"] [0]
            post2 = cameraDataRaw ["blue_posts"] [1]

            if (post1.x () < post2.x ()):
                left_post  = [post1]
                right_post = [post2]

            else:
                left_post  = [post2]
                right_post = [post1]

            cameraDataRaw.update ({"left_blue_post"  : left_post})
            cameraDataRaw.update ({"right_blue_post" : right_post})

        elif (posts_num == 1):
            post = cameraDataRaw ["blue_posts"] [0]

            if (support_num == 1):
                support = cameraDataRaw ["white_posts_support"] [0]

                if (post.x() > support.x()):
                    cameraDataRaw.update ({"left_blue_post"  : []})
                    cameraDataRaw.update ({"right_blue_post" : [post]})
                    print ("right post")

                else:
                    cameraDataRaw.update ({"left_blue_post"  : [post]})
                    cameraDataRaw.update ({"right_blue_post" : []})
                    print ("left post")

            if (support_num == 0):
                cameraDataRaw.update ({"left_blue_post"  : [post]})
                cameraDataRaw.update ({"right_blue_post" : []})

        cameraDataApproved = cameraDataRaw

        # model part. Mapping to world coords.

        # self means in robots coords
        for observationType in cameraDataApproved:
            selfData[observationType] = []
            selfPoints = []
            for observation in cameraDataApproved[observationType]:
                selfPoints.append(
                    model.pic2r(observation[0] + observation[2]/2,
                    observation[1] + observation[3]))
<<<<<<< HEAD
                selfData[observationType].extend(selfPoints)


        #print("keys = ", selfData.keys())

    #break
    loc.update(selfData)
    loc.update_ball(selfData)
    print("my_pose", loc.robot_position)
    action = strat.generate_action(loc)
    print(action)
    print(loc.pf.token)
=======
            selfData[observationType] = selfPoints
        #print(selfData)

    #break

    #loc.update(selfData)

    action = strat.generate_action(loc)

>>>>>>> 280988b1d4785684fb35834ac83790085a67c75d
    #motion.apply(action)

