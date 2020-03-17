import math
import json
try:
    import sensor
    import image
except Exception:
    raise Exception("Try to import OpenMV library using Python3")
from .detectors import SurroundedObjectDetector, blob_area, blob_width


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

                    sur1_th = (int (detector ["sthl1"]), int (detector ["sthh1"]),
                              int (detector ["sthl2"]), int (detector ["sthh2"]),
                              int (detector ["sthl3"]), int (detector ["sthh3"]))

                    alternative_surr = int (detector ["alternative surrounding"])

                    sur2_th = sur1_th

                    if (alternative_surr == 1):
                        sur2_th = (int (detector ["asthl1"]), int (detector ["asthh1"]),
                                   int (detector ["asthl2"]), int (detector ["asthh2"]),
                                   int (detector ["asthl3"]), int (detector ["asthh3"]))

                    rad_coeff      = float (detector ["rad coeff"])
                    sector_radius  = int (detector ["sector radius"])
                    circle_y_shift = float (detector ["circle y shift"])

                    window_size    = int (detector ["window size"])
                    pixel_th       = int (detector ["pixel th"])
                    area_th        = int (detector ["area th"])
                    merge_str      = detector ["merge"]

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

                    height_width_ratio_low_th  = float (detector ["height width ratio low"])
                    height_width_ratio_high_th = float (detector ["height width ratio high"])

                    new_detector = SurroundedObjectDetector(obj_th, sur1_th, sur2_th, sector_radius,
                        window_size, pixel_th, area_th, merge, point_num, min_angle, max_angle,
                        obj_num, corr_ratio, sort_func_str, roundness_th, height_width_ratio_low_th,
                        height_width_ratio_high_th, rad_coeff, circle_y_shift)

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


