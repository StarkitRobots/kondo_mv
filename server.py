import sys
import time
import math
import json
import os
import warnings
try:
    import sensor
    import image
    import pyb
    from pyb import LED
except ImportError:
    from src.simulation import sensor
    from src.simulation import image
    warnings.warn("CV reload imported")

from src.localization.tools import median
from src.vision import Vision, VisionPostProcessing
from src.strategy import Strategy
from src.motion import Motion
from src.localization import Localization
from src.model import KondoMV

class Server:
    def __init__(self, no_vision=False):
        self.no_vision = no_vision
        if not self.no_vision:
            try:
                sensor.reset()
                sensor.set_pixformat(sensor.RGB565)
                sensor.set_framesize(sensor.QVGA)
                sensor.set_auto_exposure(False)
                sensor.set_auto_whitebal(False)
                sensor.skip_frames(time=2000)
                sensor.set_auto_gain(False, gain_db=0)
                sensor.set_auto_whitebal(False, (-6.02073, -5.11, 1.002))
                sensor.set_auto_exposure(False, 2000)
            except AttributeError:
                raise Exception("No sensor found. Set 'no_vision' parameter")

            self.vision = Vision({})
            self.vision_postprocessing = VisionPostProcessing()
            self.vision.load_detectors("vision/detectors_config.json")
            with open("calibration/cam_col.json") as f:
                calib = json.load(f)

        # TODO: add side definig method
        side = "left"

        self.localization = Localization(-0.7, -1.3, math.pi/2, side)
        self.strategy = Strategy()
        self.model = KondoMV()
        self.motion = Motion(self.model)
        

        # setting model parametrs
        mass1 = [0, 0, 0, 0, 0, 0]
        mass2 = [0, 0]
        self.model.setParams(calib["cam_col"], self.model.robot_height, mass1, mass2)




    
    def run(self):
        while True:
            if not self.no_vision:
                self.clock = time.clock()
                self.clock.tick()
                curr_t = pyb.millis()
                #print (curr_t - t)
                t = curr_t
            selfData = {}
            for i in range(12):
                # motion part. Head movement.
                self.motion.apply({'name': 'head'})
                if not self.no_vision:
                    self.model.update_camera_pan_tilt(self.motion.head.pan, self.motion.head.tilt)
                    # vision part. Taking picture.
                    img = sensor.snapshot().lens_corr(strength=1.2, zoom=1.0)

                    cameraDataRaw = self.vision.get(img, objects_list=["yellow_posts", "blue_posts", "ball"],
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

                            selfPoints.append(self.model.pic2r(cx, cy + (w+h)/4))
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

            self.localization.update(selfData)
            self.localization.update_ball(selfData)
            self.localization.localized = True  #can be turned off, but now thats needs for debug
            # print(loc.ballPosSelf)

            action = self.strategy.generate_action(self.localization)
            print(action)#can be turned off, but now thats needs for debug

            # print(loc.pf.token)

            self.motion.apply(action)

            self.localization.move(self.motion.odometry.get_shift_x(),
                                    self.motion.odometry.get_shift_y(),
                                    self.motion.odometry.get_shift_yaw())