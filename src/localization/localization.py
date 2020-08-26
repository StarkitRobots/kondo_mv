import json
from .button import Button
from .tools import median
from .particle_filter import ParticleFilter
from .field import Field
from .tools import Random
from .entity import Entity
from .pf_logger import PFlogger


class Localization:

    def __init__(self, x, y, yaw, side='yellow', button=False):
        self.local_ball_coord = None
        self.global_ball_cord = None
        self.localized = False
        self.see_ball = False
        with open("localization/landmarks.json", "r") as f:
            landmarks = json.loads(f.read())
        # choice side
        if side == 'blue':
            colors = []
            for goal_color in landmarks:
                colors.append(goal_color)
            neutral = landmarks[colors[0]]
            landmarks[colors[0]] = landmarks[colors[1]]
            landmarks[colors[1]] = neutral
        if button:
            self.robot_position = tuple(Button())
            x, y, yaw = self.robot_position
        self.pf = ParticleFilter(Entity(x, y, yaw),
                                 Field("localization/parfield.json"),
                                 landmarks)

    def update(self, data):
        # updating the filter based on data from vision
        self.robot_position = self.pf.update(data)
        if self.pf.consistency > 0.5:
            self.localized = True
        else:
            self.localized = False

    def update_ball(self, data):
        # updating global ball position, if the ball is found in the frame
        if len(data['ball']) != 0:
            self.see_ball = True
            self.local_ball_coord = median(data["ball"])
            self.global_ball_cord = \
                self.pf.robot.local_to_global_coord(self.local_ball_coord)
        else:
            self.see_ball = False

    def move(self, odometry):
        self.pf.particles_move(odometry)
