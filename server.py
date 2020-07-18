import math

class Server:
    def __init__(self, no_vision=False):
        self.no_vision = no_vision
    
    def run(self):
        while True:
            clock.tick()
            curr_t = pyb.millis()
            #print (curr_t - t)
            t = curr_t
            selfData = {}
            for i in range(12):
                # motion part. Head movement.
                motion.apply({'name': 'head'})
                if not self.no_vision:
                    model.updateCameraPanTilt(motion.head.pan, motion.head.tilt)
                    # vision part. Taking picture.
                    img = sensor.snapshot().lens_corr(strength=1.2, zoom=1.0)

                    cameraDataRaw = vision.get(img, objects_list=["yellow_posts", "blue_posts", "ball"],
                                            drawing_list=["yellow_posts", "blue_posts", "ball"])

                    # cameraDataRaw=vision.get(img, objects_list=["yellow_posts", "ball", "white_posts_support"],
                    #                          drawing_list=["yellow_posts", "ball", "white_posts_support"])

                    # vision_postprocessing.process (cameraDataRaw, "yellow_posts", "white_posts_support")
                    cameraDataProcessed = cameraDataRaw
                    # model part. Mapping to world coords.

                    # self means in robots coords
                    for observationType in cameraDataProcessed:
                        if observationType not in selfData.keys():
                            selfData[observationType] = []
                        selfPoints = []

                        # if (len (cameraDataProcessed[observationType]) > 0):
                        #    print ("obser", cameraDataProcessed[observationType] [0])

                        for observation in cameraDataProcessed[observationType]:
                            cx = observation[5]
                            cy = observation[6]
                            w = observation[2]
                            h = observation[3]

                            selfPoints.append(model.pic2r(cx, cy + (w+h)/4))
                        selfData[observationType] += selfPoints

            print("eto self yello points", #can be turned off, but now thats needs for debug
                selfData['yellow_posts'], "eto self blue points", selfData["blue_posts"])

            if len(selfData['yellow_posts']) != 0:
                general = []
                first_side = []
                second_side = []
                k = selfData['yellow_posts'][0]
                for pep in selfData['yellow_posts']:
                    if math.fabs(math.atan(pep[0]/pep[1]) - math.atan(k[0]/k[1])) < 0.3:
                        first_side.append(list(pep))
                    else:
                        second_side.append(list(pep))
                if len(first_side) != 1:
                    first_side = median(first_side)
                else:
                    first_side = first_side[0]

                if second_side != 0 and len(second_side) != 1:
                    second_side = median(second_side)
                elif len(second_side) == 1:
                    second_side = second_side[0]

                general.append(first_side)
                if len(second_side) != 0:
                    general.append(second_side)
                selfData['yellow_posts'] = general
            print("eto self yello points", #can be turned off, but now thats needs for debug
                selfData['yellow_posts'], "eto self blue points", selfData["blue_posts"])

            loc.update(selfData)
            loc.update_ball(selfData)
            loc.localized = True  #can be turned off, but now thats needs for debug
            # print(loc.ballPosSelf)

            action = strategy.generate_action(loc)
            print(action)#can be turned off, but now thats needs for debug

            # print(loc.pf.token)

            motion.apply(action)

            #loc.move({'shift_x': odometry.get_odometry)