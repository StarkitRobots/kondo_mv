import math
import json

from .detector import Detector, blob_area, blob_width

class ColoredObjectDetector(Detector):
    """Color-based detector with filtration by pixel area, bbox area and blob roundness.
Maximum objects number is regulated by objects_num_, passed into the constructor
and by default is set to 1. Blob merging is supported, but is disabled by default.
Draws the confirmed detections in a verbose manner and the rest of the candidates
with single bbox. Supports custom sorting criterion, set to be blob.area() by
default.
    """
    def __init__(self, th_, pixel_th_=300, area_th_=300, merge_=False, objects_num_=1,
                 roundness_th_=-1):
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
            #print ("roundness ", blob.roundness())
            if (blob.roundness () > roundness_th):
                result.append (blob)

        return result

    def get_k_first_sorted (self, blobs, sorting_func=blob_area, k=-1):
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