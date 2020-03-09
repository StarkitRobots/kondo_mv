# Untitled - By: Robocup - Пн янв 27 2020

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()
k = 0
while(True):
    clock.tick()
    img = sensor.snapshot().lens_corr(strength=1.5, zoom = 1.0)
    img.save('calibration/images/kek'+str(k)+'.jpg')
    k +=1
    time.sleep(250)
    print(clock.fps())
