# Untitled - By: a - Ср окт 30 2019

import sensor, image, time, kondo, pyb
from kondo import Kondo
from machine import I2C
from bno055 import BNO055, AXIS_P7

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)


def soccer_walk(kondo, steps):
    i2c = I2C(2)
    imu = BNO055(i2c)


    PROPORTION = 4

    clock = time.clock()
    yaw1, roll, pitch = imu.euler()
    if yaw1 > 180: yaw1 = yaw1-360
    # 22 фрейма исходящий шаг, 32 фрейма цикл, 22 фрейма заключительный шаг.
    a = kondo.rcb4.setUserParameter(1,0)
    a = kondo.rcb4.setUserCounter( 1, steps )
    a = kondo.rcb4.motionPlay(8)
    clock.tick()
    tim1 = 220+220+320*steps
    while (clock.avg() < tim1):
        pyb.delay(200)
        yaw, roll, pitch = imu.euler()
        if yaw > 180: yaw = yaw-360
        correction = int(PROPORTION * ( yaw1 - yaw ))
        if correction > 800 :
            correction = 800
        if correction < -800 :
            correction = -800
        a = kondo.rcb4.setUserParameter(1,correction)
        print (correction, yaw1, yaw)






if __name__ == "__main__":
    uart = pyb.UART(1, 115200, parity=0)
    kondo = kondo.Kondo()
    kondo.init(uart)
    steps =5
    soccer_walk(kondo, steps)

