import sensor
import image
import time
import math
import json
from Model import Model

robotHeight = 0.28  # [m]
# init code
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()


class Detector:
    def __init__(self):
        self.blobs = []

    def detect(self, img):
        return (5, 6)

    def draw(self, img):
        for blob in self.blobs:
            print(blob.rect())
            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))


class colored_object_detector (Detector):
    def __init__(self, th_, pixel_th_=300, area_th_=300, merge_=False):
        self.th = th_
        self.pixel_th = pixel_th_
        self.area_th = area_th_
        self.merge = merge_

    def _detect(self, img):
        detected_blobs = img.find_blobs(
            [self.th], pixels_threshold=300, area_threshold=300, merge=False)

        return detected_blobs

    def detect(self, img):
        self.blobs = self._detect(img)

        return self.blobs


class Azer_ball_detector (colored_object_detector):
    def __init__(self):
        self.threshold_index = 0

        self.thresholds = [(30, 80, 0, 40, -10, 20),
                           (30, 100, -64, -8, -32, 32),
                           (0, 30, 0, 64, -128, 0)]

        self.blobs = []

    def detect(self, img):
        self.blobs = []

        detected_blobs = img.find_blobs([self.thresholds[self.threshold_index]],
                                       pixels_threshold=200, area_threshold=200, merge=True)

        for blob in detected_blobs:
            # These values depend on the blob not being circular - otherwise they will be shaky.
            if blob.roundness() > 0.9:
                print(blob.rect())
                a = image.rgb_to_lab(img.get_pixel(blob.x(), blob.y()))
                b = image.rgb_to_lab(img.get_pixel(
                    blob.x()+blob.w()-1, blob.y()))
                c = image.rgb_to_lab(img.get_pixel(
                    (blob.x()+blob.w()-1), (blob.y()+blob.h()-1)))
                d = image.rgb_to_lab(img.get_pixel(
                    blob.x(), blob.y()+blob.h()-1))

                if (all(thresholds[1][2*i] < a[i] < thresholds[1][2*i+1] for i in range(len(a)-1)) or
                    all(thresholds[1][2*i] < b[i] < thresholds[1][2*i+1] for i in range(len(b)-1)) or
                    all(thresholds[1][2*i] < c[i] < thresholds[1][2*i+1] for i in range(len(c)-1)) or
                        all(thresholds[1][2*i] < d[i] < thresholds[1][2*i+1] for i in range(len(d)-1))):
                    self.blobs.append(blob)

        return self.blobs


class Vision:
    def __init__(self, detectors_):
        self.detectors = detectors_

    def get(self, img, objects_list, drawing_list=[]):
        result = {}

        for obj in objects_list:
            detection_result = self.detectors[obj].detect(img)
            result.update({obj: detection_result})

            if (obj in drawing_list):
                self.detectors[obj].draw(img)

        return result


class Localization:
    robotPosision = (0, 0)  # ([m], [m])
    robotYaw = 0  # [<deg/rad>]
    ballPositionWorld = 0
    ballPositionSelf = 0

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


vision = Vision(
    {"ball": colored_object_detector((30, 80, 0, 40, -10, 20)),
     "blue_posts": colored_object_detector((30, 80, 0, 40, -10, 20),
     "red_posts": colored_object_detector((30, 80, 0, 40, -10, 20)})

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
        img, objects_list=["ball", "blue_posts", "red_posts"], drawing_list=["ball", "blue_posts", "red_posts"])

    # self means in robots coords
    selfData={}
    for observationType in cameraData:
        selfPoints = []
        for observation in cameraData["observationType"]:
            selfPoints.append(model.pic2r(el[0] - el[2]/2, el[1] - el[3])
        selfData[observationType] = selfPoints

    loc.ballPositionWorld = selfData["ball"]
    loc.robotPosision = updatePF(selfData)

    action=strat.generate_action(loc)

    motion.apply(action)
