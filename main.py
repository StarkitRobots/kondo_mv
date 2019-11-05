import sys
import sensor, image
import time, math, json
import os
import pyb

sys.path.append('/model')
from model import Model
sys.path.append('/localization')
from localization import Localization
sys.path.append('/motion')
from motion import Motion
sys.path.append('/strategy')
from strategy import Strategy
sys.path.append('/lowlevel')
#from lowlevel import *
sys.path.append('/vision')
from vision import *#Vision, Detector, ColoredObjectDetector, BallDetector, SurroundedObjectDetector

robotHeight = 0.37 #[m]
# init code
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

field = (30, 85, -55, 25, -15, 55)

vision = Vision({#"ball": ColoredObjectDetector((30, 80, 0, 40, -10, 20)),
    #"blue_posts": ColoredObjectDetector((20, 55, 40, 80, 30, 70)),

    #(self, obj_th_, surr_th_, sector_rad_ = 50, wind_sz_ = 3,
    #pixel_th_ = 300, area_th_ = 300, merge_ = False,
    #points_num_ = 10, min_ang_ = 0, max_ang_ = 2, objects_num_ = 1):

    #"blue_posts": SurroundedObjectDetector((0, 20, -10, 30, -45, 10),
    #                                   (40, 60, -60, -10, 0, 45),
    "blue_posts": SurroundedObjectDetector((15, 30, 25, 60, -80, -40),
                                          field,
                                       sector_rad_ = 50,
                                           min_ang_ = 0,
                                           max_ang_ = 3.14,
                                           points_num_ = 7,
                                           objects_num_ = 2),

    "white_posts_support": SurroundedObjectDetector((80, 100, -10, 50, -40, 3),
                                          field,
                                       sector_rad_ = 30,
                                           min_ang_ = 0,
                                           max_ang_ = 3.14,
                                           points_num_ = 7,
                                           objects_num_ = 1,
                                           sorting_func_=blob_width),

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
model.updateCameraPanTilt(0, -3.1415/4)

#class Vision_postprocessing:
#    def __init__(self, ):
#        self.detectors = detectors_

#    def approve(self, preliminary_result):
#        result = {}

#        for obj in objects_list:
#            detection_result = self.detectors[obj].detect(img)
#            result.update({obj: detection_result})

#            if (obj in drawing_list):
#                self.detectors[obj].draw(img)

#        return result

#vision_postprocessing = Vision_postprocessing ("blue_posts", ["left_blue_post", "right_blue_post"])

t = 0
# main loop
while(True):
    clock.tick()

    curr_t = pyb.millis()
    #print (curr_t - t)
    t = curr_t

    for i in range(motion.head_state_num):

        # motion part. Head movement.
        motion.move_head()
        model.updateCameraPanTilt(0, motion.head_yaw)
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
        selfData={}
        for observationType in cameraDataApproved:
            selfPoints = []
            for observation in cameraDataApproved[observationType]:
                selfPoints.append(
                    model.pic2r(observation[0] + observation[2]/2,
                    observation[1] + observation[3]))
            selfData[observationType] = selfPoints
        print(selfData)

    #break
    #loc.update(selfData)
    loc.update_ball(selfData)
    action = strat.generate_action(loc)
    print(action)
    #motion.apply(action)

