import math
import json
	

def blob_area(blob):
    return blob.area()

def blob_width(blob):
    return blob.w()

class Detector:
    """Implements basic visualization for the blobs and contains placeholders for the
common methods shared by all the detectors.
    """

    def __init__(self):
        self.blobs = []

    def detect(self, img):
        return (5, 6)

    def _draw(self, img, blobs=[], edges=True, axis_lines=False):
        #if (blobs == []):
        #    blobs = self.blobs

        for blob in blobs:
            if edges == True:
                img.draw_edges(blob.min_corners(), color=(255, 0, 0))

            if axis_lines == True:
                #print ("ff")
                img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))

    def draw(self, img):
        self._draw(img)