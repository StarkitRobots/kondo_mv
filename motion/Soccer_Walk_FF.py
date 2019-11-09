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


    PROPORTION = 7

    z,a1=kondo.rcb4.getUserParameter(19)
    pyb.delay(100)
    z,b1=kondo.rcb4.getUserParameter(20)
    pyb.delay(100)
    a=kondo.rcb4.setUserParameter(19,0)
    pyb.delay(100)
    a=kondo.rcb4.setUserParameter(20,0)
    pyb.delay(100)
    clock = time.clock()
    yaw1, roll, pitch = imu.euler()
    if yaw1 > 180: yaw1 = yaw1-360
    # 22 фрейма исходящий шаг, 32 фрейма цикл, 22 фрейма заключительный шаг.
    while( kondo.rcb4.getMotionPlayNum() !=0) : pyb.delay(1000)
    a = kondo.rcb4.setUserParameter(1,0)
    pyb.delay(100)
    a = kondo.rcb4.setUserCounter( 1, steps )
    pyb.delay(100)
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
    pyb.delay(100)
    a=kondo.rcb4.setUserParameter(19,a1)
    pyb.delay(100)
    a=kondo.rcb4.setUserParameter(20,b1)
    pyb.delay(100)






if __name__ == "__main__":
    uart = pyb.UART(1, 115200, parity=0)
    kondo = kondo.Kondo()
    kondo.init(uart)
    steps =10
    soccer_walk(kondo, steps)

