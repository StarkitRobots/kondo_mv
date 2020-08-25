# Untitled - By: Robocup - Пн янв 27 2020

import time
import warnings
try:
    import sensor
    import image
    import pyb
    from pyb import LED
except ImportError:
    from src.simulation import sensor
    from src.simulation import image
    warnings.warn("CV reload imported")

clock = time.clock()


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_exposure(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False, gain_db = 0)
sensor.set_auto_whitebal(False, (-6.02073, -5.11, 1.002))
sensor.set_auto_exposure(False, 1800)

sensor.skip_frames(time = 5000)

k = 0
while(True):
    clock.tick()
    img = sensor.snapshot().lens_corr(strength=1.2, zoom = 1.0)
    img.save('calibration/images/kek'+str(k)+'.jpg')
    k +=1
    time.sleep(250)
    print(clock.fps())
