import sensor
import image
import time
import math

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
        pass

    def detect(self, img):
        return (5, 6)

    def draw(self, img):
        #print ("Detector of (5, 6) has detected (5, 6)")
        pass


class Azer_ball_detector (Detector):
    def __init__(self):
        self.threshold_index = 0

        self.thresholds = [(45, 75, 24, 85, 1, 70),
                           (30, 100, -64, -8, -32, 32),
                           (0, 30, 0, 64, -128, 0)]

        self.blobs = []

    def detect(self, img)
        self.blobs = []

        detected_blobs = img.find_blobs([thresholds[threshold_index]],
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

        def draw(self, blobs):
            for blob in self.blobs:
                img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))


class Vision:
    __init__(self, detectors_):
        self.detectors = detectors_

    def get(self, img, objects_list, drawing_list=[]):
        result = {}

        for object in objects_list:
            detection_result = self.detectors[object].detect(img)
            result.update({object: detection_result})

            if (object in drawing_list):
                self.detectors[object].draw(img)

        return result

class Model:
    def __init__(self):
        self.transformMatrix = 0
    
    def update(self):
        self.transformMatrix = calcTransformMatrix()

    def cameraToSelf(self, cameraData):
        self.update()
        selfData = {}
        for observation in cameraData:
            selfData[observation] = cameraData[observation] * self.transformMatrix
        


vision = Vision()
loc = Localization()
strat = Strategy()
motion = Motion()
model = Model()
# main loop
while(True):
    clock.tick()
    img = sensor.snapshot()

    #camera means in image coords
    cameraData = vision.get(img, objects_list=["ball"], drawing_list= ["ball"])

    #self means in robots coords
    selfData = model.cameraToSelf(cameraData)

    loc.update(data)

    action = strategy.generate_action(loc)

    motion.apply(action)