from kondo import Kondo
import sensor, image, time, math,  kondo, pyb
from pyb import Pin
from pyb import LED
from machine import I2C
from bno055 import BNO055, AXIS_P7
from Soccer_Walk_FF import soccer_walk

def Penalty_Kick_2019(kondo, head_pos, head_dir_x):

    pin2 = Pin('P2', Pin.IN, Pin.PULL_UP)
    green_led = LED(2)

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA) # we run out of memory if the resolution is much bigger...
    sensor.skip_frames(time = 2000)
    sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
    #sensor.set_windowing((48,24,224,168))
    clock = time.clock()

    ala = 0

    thresholds = [(41, 100, 49, 81, -7, 60)] #мяч

    sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...

    while(ala==0):
        if (pin2.value()== 0):   # нажатие на кнопку на голове
            ala = 1
            print("нажато")
        img = sensor.snapshot().histeq(adaptive=True, clip_limit=3)
        blobs = img.find_blobs([thresholds[0]], pixels_threshold=50, area_threshold=50, merge=True, margin=10)
        if (len (blobs) == 1):
            blob = blobs [0]
            green_led.on()  # подмигивание зеленым светодиодом
            time.sleep(50)  #
            green_led.off() #
            img.draw_rectangle(blob.rect())
    i2c = I2C(2)
    imu = BNO055(i2c)
    yaw1, roll, pitch = imu.euler()
    if yaw1 > 180: yaw1 =  yaw1-360
    #else: yaw1 = - yaw1
    with open("calibr.txt", 'r') as f:
        b = int(f.read())
    a=kondo.rcb4.setUserParameter(19,0)
    a=kondo.rcb4.setUserParameter(20,b)
    #U19 - Шея поворот
    #U20 - Шея Наклон
    kondo.rcb4.motionPlay(2)
    pyb.delay(100)
    tr = 0
    #seek_Ball_In_Pose(kondo, sensor, thresholds, green_led)

    while(True):
        a, course, distance, blob = find_Ball(kondo, sensor, thresholds, green_led, i2c,imu)
        turn_To_Course(kondo,i2c,imu, course)
        soccer_walk(kondo, math.floor((distance)/8.6))
        a, course, distance, blob = find_Ball(kondo, sensor, thresholds, green_led, i2c,imu)
        #print ('course = ', course)
        while ( course > 5 or course < -5) :
            pyb.delay(100)
            a = kondo.rcb4.setUserCounter( 1, 1 )
            if course > 5 : a = kondo.rcb4.motionPlay(12)
            if course < -5 : a = kondo.rcb4.motionPlay(13)
            pyb.delay(2000)
            a=kondo.rcb4.setUserParameter(19,0)
            a=kondo.rcb4.setUserParameter(20,-2000)
            a, course, distance, blob = find_Ball(kondo, sensor, thresholds, green_led, i2c,imu)
        while (  distance > 6 ):
            a, course, distance, blob = find_Ball(kondo, sensor, thresholds, green_led, i2c,imu)
            a = kondo.rcb4.setUserCounter( 1, 1 )
            a=kondo.rcb4.setUserParameter(1,0)
            a = kondo.rcb4.motionPlay(61)
            pyb.delay(4000)
        yaw, roll, pitch = imu.euler()
        if yaw > 180: yaw = yaw -360
        #else: yaw = - yaw
        if yaw > yaw1:
            x= int(math.floor((yaw-yaw1)/12.6))
            if x >0:
                a = kondo.rcb4.setUserCounter( 1, x )
                a = kondo.rcb4.motionPlay(16)
                pyb.delay(200+ 2000*x)
        if yaw < yaw1 :
            x= int(math.floor((yaw1-yaw)/12.6))
            if x >0:
                a = kondo.rcb4.setUserCounter( 1, x )
                a = kondo.rcb4.motionPlay(17)
                pyb.delay(200+ 2000*x)
        while (  distance > 3 ):
            a, course, distance, blob = find_Ball(kondo, sensor, thresholds, green_led, i2c,imu)
            a = kondo.rcb4.setUserCounter( 1, 1 )
            a=kondo.rcb4.setUserParameter(1,0)
            a = kondo.rcb4.motionPlay(61)
            pyb.delay(4000)
        a=kondo.rcb4.setUserParameter(19,0)
        a=kondo.rcb4.setUserParameter(20,-2000)
        img = sensor.snapshot().histeq(adaptive=True, clip_limit=3)
        blobs = img.find_blobs([thresholds[0]], pixels_threshold=200, area_threshold=200, merge=True, margin=10)
        if (len (blobs) == 1):
            blob = blobs [0]
            green_led.on()  # подмигивание зеленым светодиодом
            time.sleep(50)  #
            green_led.off() #
            img.draw_rectangle(blob.rect())
            if blob.cx() < 160 :
                a = kondo.rcb4.motionPlay(19)
                pyb.delay(4000)
            if blob.cx() > 160 :
                a = kondo.rcb4.motionPlay(19)
                pyb.delay(4000)

        clock.tick()




def find_Ball(kondo, sensor, thresholds, green_led,i2c,imu):
    yaw1, roll, pitch = imu.euler()
    if yaw1 > 180: yaw1 = yaw1 -360
    #else: yaw1 = - yaw1
    a, course, distance, blob = seek_Ball_In_Frame(kondo, sensor, thresholds, green_led)
    if a == True: return True, course, distance, blob
    a, course, distance, blob = seek_Ball_In_Pose(kondo, sensor, thresholds, green_led)
    if a== False :
        turn_To_Course(kondo,i2c,imu, -160)
        a, course, distance, blob = seek_Ball_In_Pose(kondo, sensor, thresholds, green_led)
        if a == True: return True, course, distance, blob
        else:
            turn_To_Course(kondo,i2c,imu, 160)
            return False, 0, 0 , 0

    return True, course, distance, blob

def turn_To_Course(kondo,i2c,imu, course):
    yaw1, roll, pitch = imu.euler()
    if yaw1 > 180: yaw1 = yaw1 -360
    #else: yaw1 = - yaw1
    yaw2 = yaw1 + course
    while (math.fabs(course) > 3):
        while ( math.fabs(course)) > 18:
            x= math.floor(course/18)
            c1 = int(math.fabs(x))
            y= int(math.copysign(140, course))
            a = kondo.rcb4.setUserParameter(1,y)
            a = kondo.rcb4.setUserCounter( 1, c1 )
            a = kondo.rcb4.motionPlay(15)
            pyb.delay(c1*520+380)
            yaw, roll, pitch = imu.euler()
            if yaw > 180: yaw = yaw -360
            #else: yaw = - yaw
            course = yaw2 - yaw
        a = kondo.rcb4.setUserParameter(1,int(course*8.333))
        a = kondo.rcb4.setUserCounter( 1, 1 )
        a = kondo.rcb4.motionPlay(15)
        pyb.delay(1000)
        yaw, roll, pitch = imu.euler()
        if yaw > 180: yaw = yaw -360
        #else: yaw = - yaw
        course = yaw2 - yaw



def seek_Ball_In_Pose(kondo, sensor, thresholds, green_led):
    # U19 - Шея поворот
    # U20 - Шея Наклон
    with open("calibr.txt", 'r') as f:
        c = int(f.read())
    head_pose = [(-1333, c), (-2667,c), (-2667, c-593), (-1333, c-593), (0, c-593), (1333,c-593),(2667, c-593), (2667, c-1186), (1333, c-1186), ( 0, c-1186), (-1333, c-1186), (-2667, c-1186), (2667,c), (1333, c), ( 0, c) ]
    for i in range(len(head_pose)):
        x = head_pose[i]
        kondo.rcb4.setUserParameter(19,x[0])
        pyb.delay(200)
        kondo.rcb4.setUserParameter(20,x[1])
        pyb.delay(200)
        a, course, distance, blob = seek_Ball_In_Frame(kondo, sensor, thresholds, green_led)
        if a == True:
            kondo.rcb4.setUserParameter(19,int(course/0.03375))
            pyb.delay(200)
            kondo.rcb4.setUserParameter(20,int((math.atan(distance/41)*57.32-62)/0.03375+c))
            pyb.delay(200)
            return True, course, distance, blob
    return False, 0, 0, 0


def seek_Ball_In_Frame(kondo, sensor, thresholds, green_led):
    tra = 0
    for number in range (10):
        img = sensor.snapshot().histeq(adaptive=True, clip_limit=3)
        blobs = img.find_blobs([thresholds[0]], pixels_threshold=50, area_threshold=50, merge=True, margin=10)
        if (len (blobs) == 1):
            blob = blobs [0]
            green_led.on()  # подмигивание зеленым светодиодом
            time.sleep(50)  #
            green_led.off() #
            tra = tra + 1
            img.draw_rectangle(blob.rect(), color = (255, 0, 0))
            course, distance = get_course_and_distance(kondo, blob)
            return True, course, distance, blob
    if tra == 0: return False, 0, 0, 0

def get_course_and_distance(kondo, blob):
    z,a=kondo.rcb4.getUserParameter(19)
    pyb.delay(100)
    z,b=kondo.rcb4.getUserParameter(20)
    # U19 - Шея поворот
    # U20 - Шея Наклон
    with open("calibr.txt", 'r') as f:
        c = int(f.read())
    course = a*0.03375 - (160 - blob.cx())*0.1875
    x = 62 + (b-c)*0.03375 +(120-blob.cy())*0.1875
    y=math.radians(x)
    distance = 41* math.tan(y)
    return course, distance






if __name__ == "__main__":
    uart = pyb.UART(1, 115200, parity=0)
    kondo = kondo.Kondo()
    kondo.init(uart)

    thresholds = [(41, 100, 49, 81, -7, 60)] #мяч

    head_pos_x = 0 #  от -2 до 2
    head_pos_y = 0 # от -2 до 0
    head_dir_x = 1 # 1 - вправо, -1 - влево
    head_dir_y = -1 # -1 - вниз, 1 - вверх
    head_pos = 1
    Penalty_Kick_2019(kondo, head_pos, head_dir_x)
    #turn_To_Course(kondo, 170)
