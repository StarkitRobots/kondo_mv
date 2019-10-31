import sys 
import sensor, image
import time, math, json

from model.model import Model
from localization.localization import Localization
from motion.motion import Motion
from strategy.strategy import Strategy
from vision.vision import Vision, Detector, ColoredObjectDetector, BallDetector


robotHeight = 0.41 #[m]
# init code
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()


vision = Vision({"ball": ColoredObjectDetector((30, 80, 0, 40, -10, 20)),
    "blue_posts": ColoredObjectDetector((20, 55, 40, 80, 30, 70)),
    "yellow_posts": ColoredObjectDetector((20, 55, 40, 80, 30, 70))})

loc=Localization()
strat=Strategy()
motion=Motion()
model=Model()

with open("cam_col.json") as f:
    calib=json.load(f)

# setting model parametrs
model.setParams(calib["cam_col"], robotHeight)
model.updateCameraPanTilt(0, -3.1415/6)

# main loop
while(True):
    clock.tick()
    img=sensor.snapshot()

    # camera means in image coords
    cameraData=vision.get(
        img, objects_list=["ball", "blue_posts", "yellow_posts"], 
        drawing_list=["ball", "blue_posts", "yellow_posts"])

    # self means in robots coords
    selfData={}
    for observationType in cameraData:
        selfPoints = []
        for observation in cameraData["observationType"]:
            selfPoints.append(
                model.pic2r(observation[0] - observation[2]/2, 
                observation[1] - observation[3]))
        selfData[observationType] = selfPoints

    loc.update(selfData)
    
    action = strat.generate_action(loc)

    motion.apply(action)

