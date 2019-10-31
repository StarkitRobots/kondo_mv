import sys 
import sensor, image
import time, math, json

#sys.path.append('model')
from model import Model

#sys.path.append('localisation')
#from localisation import Localisation
from PatFilter4MicroPython import updatePF

sys.path.append('vision')
from vision import Vision, Detector, ColoredObjectDetector, BallDetector

robotHeight = 0.41 #[m]
# init code
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()


class Localization:
    def __init__(self):
        pass

    def update(self, data):
        return 0


class Strategy:
    def __init__(self):
        pass

    def generate_action(self, loc):
        return 0


class Motion:
    def __init__(self):
        pass

    def apply(self, action):
        return 0

vis = Vision({"goal" : ColoredObjectDetector((20, 55, 40, 80, 30, 70))})
loc = Localization()
strat = Strategy()
motion = Motion()
model = Model()

with open("cam_col.json") as f:
    calib = json.load(f)

#setting model parametrs
model.setParams(calib["cam_col"], robotHeight)
model.updateCameraPanTilt(0, -3.1415/6)

# main loop
while(True):
    clock.tick()
    img = sensor.snapshot()

    # camera means in image coords
    cameraData = vis.get(img, objects_list=["goal"], drawing_list= ["goal"])

    # self means in robots coords
    selfData = []
    for el in cameraData["goal"]:
        selfData.append(model.pic2r(el[0] - el[3]/2, el[1] - el[4])) # (el[0] - el[3]/2, el[1] - el[4])
        print("data = ", selfData[-1])

    loc.ballPositionWorld = selfData["ball"]
    loc.robotPosision = updatePF(selfData)
    
    action=strat.generate_action(loc)

    motion.apply(action)
