from pyb import Pin
import utime

pin9 = Pin('P9', Pin.IN, Pin.PULL_UP)
pin3 = Pin('P3', Pin.IN, Pin.PULL_UP)
pin2 = Pin('P2', Pin.IN, Pin.PULL_UP)

class Button():
    def __init__():
        b_time = utime.ticks_ms()
        print(b_time)
        side_loop = 0
        side = False
        while(side_loop == 0):
            if (pin9.value() == 0):  
                side_loop = 1
                side = True
                print("I will attack blue goal")
                break
            if (utime.ticks_ms() - b_time) > 1500:
                print("I will attack yellow goal")
                break
        b_time = utime.ticks_ms()
        side_loop = 0
        s_coord = 0
        while(side_loop == 0):
            if (pin3.value() == 0):
                side_loop
                s_coord = 2
                print("right")
                break
            if (pin2.value() == 0):
                side_loop
                s_coord = 1
                print("left")
                break
            if (utime.ticks_ms() - b_time) > 1500:
                print("centr")
                break
        with open("start_coord.json", "r") as f:
            start_coord = json.loads(f.read())
        return start_coord[str(s_coord)]