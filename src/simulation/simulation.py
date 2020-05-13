import time
from . import vrep
from . import vrep_consts
from . entities import Ball, Robot, Field

class Simulation:
    def __init__(self, use_physics):
        self.use_physics = use_physics

        self.real_time = False
        self.t = 0.0
        self.dt = 0.02

        self.players = {}
        self.teams = {'left': None, 'right': None}
        self.processes = {}
        self.base_port = 3000
        
        # just in case, close all opened connections
        vrep.simxFinish(-1) 
        
        # Connect to V-REP
        self.client_id = vrep.simxStart('127.0.0.1',
                                        19997, True, True, 5000, self.dt * 1000)
        if self.client_id != -1:
            print ('Connected to remote API server')
        else:
            print ('Failed connecting to remote API server')
            print ('Program ended')
            exit(0)

        self.robot = Robot(self.client_id)
        self.ball = Ball(self.client_id)
        self.field = Field(self.client_id)

        if self.use_physics:
            vrep.simxSynchronous(self.client_id, True)

    def start(self):
        if self.use_physics:
            time.sleep(0.1)
            vrep.simxStartSimulation(self.client_id, vrep_consts.simx_opmode_oneshot)

    def get_ball_pos(self):
        ball_pos = self.ball.get_position(self.field)
        return (ball_pos[0], ball_pos[1])

    def set_ball_pos(self, new_pos):
        pos = self.ball.get_position(self.field)
        self.ball.set_position(self.field, (new_pos[0], new_pos[1], pos[2]))

    @staticmethod
    def show_fake_vision(robot):
        robot.show_fake_vision = True

    def tick(self):        
            self.robot.update(self.field)
            self.ball.update(self.field)
            self.t += self.dt
            vrep.simxSynchronousTrigger(self.client_id)

    def stop(self):
        vrep.simxStopSimulation(self.client_id, vrep_consts.simx_opmode_oneshot)

    def __del__(self):
        time.sleep(0.2)
        vrep.simxFinish(self.client_id)