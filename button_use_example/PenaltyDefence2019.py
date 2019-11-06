
import sensor, image, time, math
from kondo import Kondo
import sensor, image, time, math,  kondo, pyb
from pyb import Pin
from pyb import LED
from machine import I2C
from bno055 import BNO055, AXIS_P7

pin2 = Pin('P2', Pin.IN, Pin.PULL_UP)
green_led = LED(2)

uart = pyb.UART(1, 115200, parity=0)
kondo = kondo.Kondo()
kondo.init(uart)

a=kondo.rcb4.setUserParameter(19,0)
a=kondo.rcb4.setUserParameter(20,-700)

threshold_index = 0 # 0 for red, 1 for green, 2 for blue

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [(41, 100, 49, 81, -7, 60),
              (30, 100, -64, -8, -32, 32),
              (0, 30, 0, 64, -128, 0)]

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()
sensor.set_windowing((80,45,160, 50))


# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

ala = 0
while(ala==0):
    if (pin2.value()== 0):   # нажатие на кнопку на голове
        ala = 1
        print("нажато")
    img = sensor.snapshot().histeq(adaptive=True, clip_limit=3)
    blobs = img.find_blobs([thresholds[0]], pixels_threshold=20, area_threshold=20, merge=True, margin=10)
    if (len (blobs) > 1):
        blob = blobs [0]
        green_led.on()  # подмигивание зеленым светодиодом
        time.sleep(50)  #
        green_led.off() #
        img.draw_rectangle(blob.rect())

while(True):
    clock.tick()
    img = sensor.snapshot()
    for blob in img.find_blobs([thresholds[threshold_index]], pixels_threshold=20, area_threshold=20, merge=True):
        a = kondo.rcb4.motionPlay(24)
        pyb.delay(10000)
        break
        img.draw_edges(blob.min_corners(), color=(255,0,0))
        img.draw_line(blob.major_axis_line(), color=(0,255,0))
        img.draw_line(blob.minor_axis_line(), color=(0,0,255))
        # These values are stable all the time.
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)
    print(clock.fps())
