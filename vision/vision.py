import sensor, image
import math, json

class Detector:
    def __init__(self):
        self.blobs = []

    def detect(self, img):
        return (5, 6)

    def _draw(self, img, blobs=[], edges=True, axis_lines=False):
        #if (blobs == []):
        #    blobs = self.blobs

        for blob in blobs:
            if (edges == True):
                img.draw_edges(blob.min_corners(), color=(255, 0, 0))

            if (axis_lines == True):
                #print ("ff")
                img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))

    def draw(self, img):
        self._draw(img)

def blob_area (blob):
    return blob.area ()

def blob_width (blob):
    return blob.width ()

class ColoredObjectDetector(Detector):
    def __init__(self, th_, pixel_th_ = 300, area_th_ = 300, merge_ = False, objects_num_ = 1,
                 roundness_th_ = -1):
        self.th       = th_
        self.pixel_th = pixel_th_
        self.area_th  = area_th_
        self.merge    = merge_

        self.objects_num = objects_num_
        self.roundness_th = roundness_th_

        self.result = []

    def _detect(self, img):
        detected_blobs = img.find_blobs([self.th], pixels_threshold=self.pixel_th,
            area_threshold=self.area_th, merge=self.merge)

        return detected_blobs

    def _filter_by_roundness(self, blobs, roundness_th):
        result = []

        for blob in blobs:
            if (blob.roundness > roundness_th):
                result.append (blob)

        return result

    def get_k_first_sorted (self, blobs, sorting_func = blob_area, k=-1):
        if (k == -1):
            k = len (blobs)

        blobs_sorted = sorted (blobs, key=lambda blob: blob.area(), reverse=True)

        result = blobs_sorted [:k]

        return result

    def detect(self, img):
        self.blobs = self._detect (img)

        if (self.roundness_th != -1):
            self.blobs = self._filter_by_roundness(self.blobs, self.roundness_th)

        self.result = self.get_k_first_sorted (self.blobs, self.objects_num)

        return self.result

    def draw(self, img):
        self._draw(img, self.result, True, True)
        self._draw(img, self.blobs)

class SurroundedObjectDetector(ColoredObjectDetector):
    def __init__(self, obj_th_, surr_th_, sector_rad_ = 50, wind_sz_ = 3,
            pixel_th_ = 300, area_th_ = 300, merge_ = True,
            points_num_ = 10, min_ang_ = 0, max_ang_ = 2, objects_num_ = 1,
            corr_ratio_ = 0.5, sorting_func_ = blob_area, roundness_th_ = -1):
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

        self.corr_ratio = corr_ratio_
        self.sorting_func = sorting_func_
        self.roundness_th = roundness_th_

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
        self.check_success = []

        if (self.roundness_th != -1):
            self.blobs = self._filter_by_roundness(self.blobs, self.roundness_th)

        #get candidates
        unchecked_result = self.get_k_first_sorted (self.blobs, self.sorting_func, self.objects_num)

        #get averaged surroundings for each
        for res in unchecked_result:
            bbox = res.rect()
            x = int (bbox[0] + bbox[2]/2)
            y = int (bbox[1] + bbox[3])

            proper = 0

            curr_step_success = []

            for pt in self.sector_points:
                pixels = []

                for i in range (x + pt [0] - self.wind_sz // 2, x + pt [0] + self.wind_sz // 2 + 1):
                    for j in range (y + pt [1] - self.wind_sz // 2, y + pt [1] + self.wind_sz // 2 + 1):
                        pix = img.get_pixel(x + pt [0] + i, y + pt [1] + j)
                        if (pix is not None):
                            pix_lab = image.rgb_to_lab(pix)
                            pixels.append(pix_lab)

                if (len (pixels) > 0):
                    #print ("sas", pixels)

                    r = 0#sum (pixels [:] [0]) / len (pixels)
                    g = 0#sum (pixels [:] [1]) / len (pixels)
                    b = 0#sum (pixels [:] [2]) / len (pixels)

                    for pix in pixels:
                        r += pix [0]
                        g += pix [1]
                        b += pix [2]

                    a = (r, g, b)

                    #a = image.rgb_to_lab(pix)
                    if (all(self.surr_th[2*i] < a[i] < self.surr_th[2*i+1] for i in range(len(a)-1))):
                        proper += 1
                        curr_step_success.append (True)

                    else:
                        curr_step_success.append (False)
                else:
                    curr_step_success.append (True)
                    proper += 1

            self.check_success.append (curr_step_success)
            #self.succ_num.append (proper)

            #print (proper)

            if (proper >= self.points_num * self.corr_ratio):
                self.result.append (res)

        #self.result = unchecked_result

        return self.result

    def draw(self, img):
        #print (len (self.result))
        self._draw(img, self.result, True, True)
        self._draw(img, self.blobs)

        i = 0
        for res in self.get_k_first_sorted (self.blobs, self.sorting_func, self.objects_num):
            bbox = res.rect()
            x = int (bbox[0] + bbox[2]/2)
            y = int (bbox[1] + bbox[3])

            j = 0
            for pt in self.sector_points:
                col = (30, 70, 130)

                if (self.check_success [i] [j] == False):
                    col = (130, 100, 10)

                j += 1

                img.draw_circle (x + pt [0], y + pt [1], 3, color=col, thickness=1, fill=True)

            i += 1

class Vision:
    def __init__(self, detectors_):
        self.detectors = detectors_

    def load_detectors(self, settings_filename):
        with open (settings_filename) as f:
            data = json.load(f)

            for detector in data ["detectors"]:
                detector_name = detector ["name"]
                detector_type = detector ["type"]
                print (detector_name, detector_type)

                if (detector_type == "SurroundedObjectDetector"):
                    obj_th = (int (detector ["othl1"]), int (detector ["othh1"]),
                              int (detector ["othl2"]), int (detector ["othh2"]),
                              int (detector ["othl3"]), int (detector ["othh3"]))

                    sur_th = (int (detector ["sthl1"]), int (detector ["sthh1"]),
                              int (detector ["sthl2"]), int (detector ["sthh2"]),
                              int (detector ["sthl3"]), int (detector ["sthh3"]))

                    sector_radius = int (detector ["sector radius"])
                    window_size = int (detector ["window size"])
                    pixel_th = int (detector ["pixel th"])
                    area_th = int (detector ["area th"])
                    merge_str = detector ["merge"]

                    merge = True
                    if (merge_str == "False"):
                        merge = False

                    point_num = int (detector ["point num"])
                    min_angle = float (detector ["min angle"])
                    max_angle = float (detector ["max angle"])
                    obj_num = int (detector ["objects num"])
                    corr_ratio = float (detector ["correct ratio"])
                    sort_func_str = detector ["sorting func"]

                    sort_func = blob_area
                    if (sort_func_str == "blob_width"):
                        sort_func = blob_width

                    roundness_th = float (detector ["roundness th"])

                    new_detector = SurroundedObjectDetector(obj_th, sur_th, sector_radius,
                        window_size, pixel_th, area_th, merge, point_num, min_angle, max_angle,
                        obj_num, corr_ratio, sort_func_str, roundness_th)

                else:
                    print("unsupported detector type")

                self.add_detector(new_detector, detector_name)

    def add_detector(self, new_detector, name):
        self.detectors.update ({name : new_detector})

    def clear(self):
        self.detectors = []

    def get(self, img, objects_list, drawing_list=[]):
        result = {}

        for obj in objects_list:
            detection_result = self.detectors[obj].detect(img)
            result.update({obj: detection_result})

            if (obj in drawing_list):
                self.detectors[obj].draw(img)

        return result

class Vision_postprocessing:
    def __init__(self):
        pass

    def process(self, cameraDataRaw, posts, support):
        posts_num   = len (cameraDataRaw [posts])
        support_num = len (cameraDataRaw [support])

        left_post  = []
        right_post = []

        if (posts_num == 2):
            post1 = cameraDataRaw [posts] [0]
            post2 = cameraDataRaw [posts] [1]

            if (post1.x () < post2.x ()):
                left_post  = [post1]
                right_post = [post2]

            else:
                left_post  = [post2]
                right_post = [post1]

            cameraDataRaw.update ({"left_"  + posts  : left_post})
            cameraDataRaw.update ({"right_" + posts : right_post})

        elif (posts_num == 1):
            post = cameraDataRaw [posts] [0]

            if (support_num == 1):
                support = cameraDataRaw [support] [0]

                if (post.x() > support.x()):
                    cameraDataRaw.update ({"left_" + posts : []})
                    cameraDataRaw.update ({"right_" + posts : [post]})
                    print ("right post")

                else:
                    cameraDataRaw.update ({"left_" + posts : [post]})
                    cameraDataRaw.update ({"right_" + posts : []})
                    print ("left post")

            if (support_num == 0):
                cameraDataRaw.update ({"left_" + posts : [post]})
                cameraDataRaw.update ({"right_" + posts : []})

        cameraDataRaw.update ({"left_"  + posts : left_post})
        cameraDataRaw.update ({"right_" + posts : right_post})

        return cameraDataRaw
