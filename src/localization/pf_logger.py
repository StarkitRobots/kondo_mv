"""
This is logger for particle filter. It saves positions of
all particles and robot in every steps to log.json file. Each log entry is:
1 name of the action, 2 position of the robot, 3 position of all particles.
Record format - dictionary of dictionaries.
"""
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
        logs = {'name': name, 'position': robot, 'particles': logs}
        with open(self.path, "a") as self.logs:
            self.logs.write(self.comma+'"'+str(self.count)+'"'+":")
        with open(self.path, "a") as self.logs:
            json.dump(logs, self.logs)
            self.logs.write("}")
        self.comma = ','
        self.count += 1
