import math
import json
try:
    import sensor
    import image
except Exception:
    raise Exception("Try to import OpenMV library using Python3")
	

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
    return blob.w ()