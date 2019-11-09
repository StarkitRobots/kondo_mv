import sys
import sensor, image
import time, math, json
import os
import pyb
from pyb import Pin
from pyb import LED

def mean(l):
    if len(l) == 0:
        return 0
    tmp = 0
    for el in l:
        tmp+=el
    return tmp/len(l)

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

pin9 = Pin('P9', Pin.IN, Pin.PULL_UP)
pin3 = Pin('P3', Pin.IN, Pin.PULL_UP)


def __main__():
    robotHeight = 0.40 #[m]
    cameraHeight = 0.04 #[m]
    # init code
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False, 9.1)  # must be turned off for color tracking
    sensor.set_auto_whitebal(False, (0.1343897, -6.02073, 2.716595))
    sensor.set_auto_exposure(False, 6500) # must be turned off for color tracking
    clock = time.clock()

    print ("gain = ", sensor.get_gain_db())
    print("gain for white = ", sensor.get_rgb_gain_db())
    print("exp = ", sensor.get_exposure_us())

    vision = Vision ({})
    vision.load_detectors("vision/detectors_config.json")

    loc=Localization(-1.3, 0.0, 0.0)
    strat=Strategy()
    motion=Motion()
    model=Model()

    with open("calibration/cam_col.json") as f:
        calib=json.load(f)

    # setting model parametrs
    model.setParams(calib["cam_col"], robotHeight)
    #pan, tilt = motion.move_head()
    #model.updateCameraPanTilt(pan, tilt)

    vision_postprocessing = Vision_postprocessing ()

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
            pan, tilt = motion.move_head()
            model.updateCameraPanTilt(pan, tilt)
            # vision part. Taking picture.
            img=sensor.snapshot()

            #img.save ("kekb.jpg", quality=100)

            cameraDataRaw=vision.get(img, objects_list=["blue_posts", "ball"],#, "white_posts_support"],
                             drawing_list=["blue_posts", "ball"])#, "white_posts_support"])

            cameraDataProcessed = cameraDataRaw#vision_postprocessing.process (cameraDataRaw, "blue_posts", "white_posts_support")
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
            time.sleep(200) # sleep for better controlling vision, not for prod

        #cacl posts pan and put it in loc.postPan
        loc.update_posts(selfData)

        #clusterize posts for negative and positive pan relative to the robot
        positive_pan = []
        negative_pan = []
        for post in loc.postPan:
            if post > 0:
                positive_pan.append(post)
            else:
                negative_pan.append(post)
        negative_pan = mean(negative_pan)
        positive_pan = mean(positive_pan)
        print("negative_pan = ", negative_pan, "positive_pan = ", positive_pan)

        #new update for ball. Now it filtered too close balls(we suppose it to be sevros LED) and take the nearest from the remaining
        bestBall = (100, 100) # simply very far ball
        for el in selfData["ball"]:
            if math.sqrt(el[0]**2 + el[1]**2) < math.sqrt(bestBall[0]**2 + bestBall[1]**2) and math.sqrt(el[0]**2 + el[1]**2) > 0.1:
                bestBall = el

        #If we find propper ball, push it to localization
        if bestBall[0] != 100:
            loc.seeBall = True
            loc.ballPosSelf = bestBall
            print("best ball = ", bestBall)
        else:
            loc.seeBall = False


        if loc.seeBall:
            action = strat.walkball(loc)
            if (math.sqrt(loc.ballPosSelf[0]**2 + loc.ballPosSelf[1]**2) < 0.15):
            #if we see the ball nearby us, checking for our orintation
                if (negative_pan != 0) and (positive_pan != 0):
                    print("I see both posts")
                    #if it see both posts, lets check, can we do direct kick
                    #if not, trying to orintate body to the center of the goal
                    if (positive_pan + negative_pan < math.pi/8) or (positive_pan + negative_pan > -math.pi/8):
                        print("I'm targeted on goal")
                        action = {"name" : "kick", "args" : (1)}
                    else:
                        if positive_pan + negative_pan < 0:
                            action ={"name" : "take_around_left"}
                        else:
                            action ={"name" : "take_around_right"}
                #if we dont see one of post, trying to see
                else:
                    if negative_pan == 0:
                        action ={"name" : "take_around_left"}

                    elif positive_pan == 0:
                        action={"name" : "take_around_right"}
                #if we dont see both, trying to do forward step. MB here we can turn or smth else
                    else:
                        action={"name" : "walk", "args" : (0.1, 0)}

        else:
            action = strat.searchball(loc)

        #loc.update(selfData)
        #print("posts number = ", len(selfData["blue_posts"]))
        #print("my_pose", loc.robot_position)
        #print(loc.ballPosSelf)

        #action = strat.generate_action(img, loc, model)
        print(action)
        #print(loc.pf.token)

        odometry_results = motion.apply(action)
        #if odometry_results is not None:
        #    loc.pf.move(odometry_results)

ala = 0
while(ala==0):
    if (pin9.value()== 0):   # нажатие на кнопку на голове
        ala = 1
        print("pressed")
        __main__()

