import json
import os 


class PFlogger():
    def __init__(self, path):
        self.path = path
        with open(path, "w") as self.logs:
            self.logs.write("{ }")
            self.comma = ''
        self.count = 0


    def step(self, name, robot, particles):
        with open(self.path, 'rb+') as filehandle:
            filehandle.seek(-1, os.SEEK_END)
            filehandle.truncate()
        logs = []
        for particle in particles:
            logs.append([particle[0].x, particle[0].y, particle[0].yaw])
        logs = {'name':name, 'position':robot, 'particles':logs}
        #logs = {(self.count):logs}
        with open(self.path, "a") as self.logs:
            self.logs.write(self.comma+'"'+str(self.count)+'"'+":")
        with open(self.path, "a") as self.logs:
            json.dump(logs, self.logs)
            self.logs.write("}")
        self.comma = ','
        self.count +=1