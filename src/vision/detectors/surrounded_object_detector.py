import math
import json
import warnings
try:
    import sensor
    import image
except ImportError:
    from src.simulation import sensor
    from src.simulation import image
    warnings.warn("CV reload imported")
from .detector import blob_area, blob_width
from .colored_object_detector import ColoredObjectDetector

class SurroundedObjectDetector(ColoredObjectDetector):
    """Contains the surrounding color checking on the top of the ColoredObjectDetector
methods. Supports multiple filtering criterions (see constructor for the whole
list) and the following parameters of the surrounding check: sector for the
checking (detailed explanation below), points number.
Extends the detection to the "object + surrounding", which is practically the
case for robotic football. The number of the possible surrounding colors is 2
to handle both the field and the marking lines.
    """
    def __init__(self, obj_th_, surr1_th_, surr2_th_, sector_rad_ = 50, wind_sz_ = 3,
            pixel_th_ = 300, area_th_ = 300, merge_ = True,
            points_num_ = 10, min_ang_ = 0, max_ang_ = 2, objects_num_ = 1,
            corr_ratio_ = 0.5, sorting_func_ = blob_area, roundness_th_ = -1,
            heigh_width_ratio_low_th_ = -1, heigh_width_ratio_high_th_ = -1,
            rad_coeff_ = 1, circle_y_shift_ = 0):
        self.th  = obj_th_
        self.surr1_th = surr1_th_
        self.surr2_th = surr2_th_

        self.pixel_th = pixel_th_
        self.area_th  = area_th_
        self.merge    = merge_

        self.rad_coeff   = rad_coeff_
        self.sector_rad  = sector_rad_
        self.circle_y_shift = circle_y_shift_

        self.wind_sz     = wind_sz_
        self.points_num  = points_num_
        self.min_ang     = min_ang_
        self.max_ang     = max_ang_
        self.objects_num = objects_num_

        self.corr_ratio = corr_ratio_
        self.sorting_func = sorting_func_
        self.roundness_th = roundness_th_
        self.heigh_width_ratio_low_th  = heigh_width_ratio_low_th_
        self.heigh_width_ratio_high_th = heigh_width_ratio_high_th_

        self._generate_encl_points()

    def _generate_encl_points(self):
        self.sector_points = []
        self.angular_step = (self.max_ang - self.min_ang) / (self.points_num - 1)

        for i in range (self.points_num):
            x = int (self.sector_rad * math.cos (self.min_ang + self.angular_step * i))
            y = int (self.sector_rad * math.sin (self.min_ang + self.angular_step * i))

            self.sector_points.append ((x, y))

    def detect(self, img):
        """The surrounding color check is implemented in the following manner:
        - given number of points lying on the segment of the circle around
        the object is generated, see _generate_encl_points()
        - the circle radius is chosen with respect to the object size
        - the color of the surrounding in those points is checked to fit
        up to 2 possible color ranges
        - the fitness of the points is stored in boolean format
        - finally, the voting is performed with the threshold ratio of
        the necessary points having chosen surrounding color
        Multiple filters are sequentially allpied, resulting in the dict
        of detected objects

        Args:
            img ([type]): [description]

        Returns:
            [type]: [description]
        """
        self.blobs = self._detect (img)
        self.result = []
        self.check_success = []

        res = []

        if (self.roundness_th != -1):
            res = self._filter_by_roundness(self.blobs, self.roundness_th)

        else:
            res = self.blobs

        #print ("interm res ", res)

        if (self.heigh_width_ratio_low_th  != -1 or
            self.heigh_width_ratio_high_th != -1):
            self.blobs = []

            for blob in res:
                height_width_ratio = float (blob.h()) / blob.w()
                #print ("heiw rat ", height_width_ratio)

                if (self.heigh_width_ratio_low_th  != -1 and
                    self.heigh_width_ratio_high_th != -1):
                    if (height_width_ratio > self.heigh_width_ratio_low_th and
                        height_width_ratio < self.heigh_width_ratio_high_th):
                        self.blobs.append (blob)

                elif (self.heigh_width_ratio_low_th  != -1):
                    if (height_width_ratio > self.heigh_width_ratio_low_th):
                        self.blobs.append (blob)

                elif (self.heigh_width_ratio_high_th  != -1):
                    if (height_width_ratio < self.heigh_width_ratio_high_th):
                        self.blobs.append (blob)

        else:
            self.blobs = []
            self.blobs = res

        #get candidates
        unchecked_result = self.get_k_first_sorted (self.blobs, self.sorting_func, self.objects_num)

        if (self.rad_coeff < 0 and len (unchecked_result) != 0):
            self.sector_rad = - blob_width(unchecked_result[0]) * self.rad_coeff
            self._generate_encl_points()

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
                    if (all(self.surr1_th[2*i] < a[i] < self.surr1_th[2*i+1] for i in range(len(a)-1)) or
                        all(self.surr2_th[2*i] < a[i] < self.surr2_th[2*i+1] for i in range(len(a)-1))):
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

            if (self.circle_y_shift - int (self.circle_y_shift) != 0):
                y += int (self.circle_y_shift * bbox[2])
            else:
                #print ("poshla mazuta")
                y += int (self.circle_y_shift)

            j = 0
            for pt in self.sector_points:
                col = (30, 70, 130)

                if (self.check_success [i] [j] == False):
                    col = (130, 100, 10)

                j += 1

                img.draw_circle (x + pt [0], y + pt [1], 3, color=col, thickness=1, fill=True)

            i += 1