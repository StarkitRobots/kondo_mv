import sensor, image
import math

class Detector:
    def __init__(self):
        self.blobs = []

    def detect(self, img):
        return (5, 6)

    def _draw(self, img, blobs=[], edges=True, axis_lines=False):
        if (blobs == []):
            blobs = self.blobs

        for blob in blobs:
            if (edges == True):
                img.draw_edges(blob.min_corners(), color=(255, 0, 0))

            if (axis_lines == True):
                img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))

    def draw(self, img):
        self._draw(img)

class ColoredObjectDetector(Detector):
    def __init__(self, th_, pixel_th_ = 300, area_th_ = 300, merge_ = False, objects_num_ = 1):
        self.th       = th_
        self.pixel_th = pixel_th_
        self.area_th  = area_th_
        self.merge    = merge_

        self.objects_num = objects_num_

        self.result = []

    def _detect(self, img):
        detected_blobs = img.find_blobs([self.th], pixels_threshold=self.pixel_th,
            area_threshold=self.area_th, merge=self.merge)

        return detected_blobs

    def get_k_first_sorted (self, blobs, k=-1):
        if (k == -1):
            k = len (blobs)

        blobs_sorted = sorted (blobs, key=lambda blob: blob.area(), reverse=True)

        result = blobs_sorted [:k]

        return result

    def detect(self, img):
        self.blobs = self._detect (img)

        self.result = self.get_k_first_sorted (self.blobs, self.objects_num)

        return self.result

    def draw(self, img):
        self._draw(img, self.result, True, True)
        self._draw(img, self.blobs)

class SurroundedObjectDetector(ColoredObjectDetector):
    def __init__(self, obj_th_, surr_th_, sector_rad_ = 50, wind_sz_ = 3,
            pixel_th_ = 30, area_th_ = 30, merge_ = True,
            points_num_ = 10, min_ang_ = 0, max_ang_ = 2, objects_num_ = 1):
        self.th  = obj_th_
        self.surr_th = surr_th_

        self.pixel_th = pixel_th_
        self.area_th  = area_th_
        self.merge    = merge_

        self.sector_rad  = sector_rad_
        self.wind_sz     = wind_sz_
        self.points_num  = points_num_
        self.min_ang     = min_ang_
        self.max_ang     = max_ang_
        self.objects_num = objects_num_

        self._generate_encl_points()

    def _generate_encl_points(self):
        self.sector_points = []
        self.angular_step = (self.max_ang - self.min_ang) / (self.points_num - 1)

        for i in range (self.points_num):
            x = int (self.sector_rad * math.cos (self.min_ang + self.angular_step * i))
            y = int (self.sector_rad * math.sin (self.min_ang + self.angular_step * i))

            self.sector_points.append ((x, y))

    def detect(self, img):
        self.blobs = self._detect (img)
        self.result = []

        #get candidates
        unchecked_result = self.get_k_first_sorted (self.blobs, self.objects_num)

        #get averaged surroundings for each
        for res in unchecked_result:
            bbox = res.rect()
            x = int (bbox[0] + bbox[2]/2)
            y = int (bbox[1] + bbox[3])

            proper = 0

            for pt in self.sector_points:
                pix = img.get_pixel(x + pt [0], y + pt [1])

                if (pix is not None):
                    a = image.rgb_to_lab(pix)
                    if (all(self.surr_th[2*i] < a[i] < self.surr_th[2*i+1] for i in range(len(a)-1))):
                        proper += 1

            print (proper)

            if (proper >= self.points_num // 2):
                self.result.append (res)

        self.result = unchecked_result

        return self.result

    def draw(self, img):
        self._draw(img, self.result, True, True)
        #self._draw(img, self.blobs)

        for res in self.result:
            bbox = res.rect()
            x = int (bbox[0] + bbox[2]/2)
            y = int (bbox[1] + bbox[3])

            for pt in self.sector_points:
                img.draw_circle (x + pt [0], y + pt [1], 3, color=(30, 70, 130), thickness=1, fill=True)

class BallDetector (ColoredObjectDetector):
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

                if (all(self.thresholds[1][2*i] < a[i] < self.thresholds[1][2*i+1] for i in range(len(a)-1)) or
                    all(self.thresholds[1][2*i] < b[i] < self.thresholds[1][2*i+1] for i in range(len(b)-1)) or
                    all(self.thresholds[1][2*i] < c[i] < self.thresholds[1][2*i+1] for i in range(len(c)-1)) or
                        all(self.thresholds[1][2*i] < d[i] < self.thresholds[1][2*i+1] for i in range(len(d)-1))):
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
