import math
from utils.Vector import Vector, Quaternion
from inverse_kinematics import compute_leg_ik



class WalkEngine:
    def __init__(self, model):
        self._model = model
        self.arms_enabled = True
        # mm side amplitude 
        # (maximum distance between most right and most left position of Center of Mass) 53.4*2
        self.amplitude = 0.032
        # Distance between Center of mass and floor in walk pose
        self.gait_height = 0.018         
        # elevation of sole over floor
        self.step_height = 0.032       
        
        # frame number for 1-st phase of gait (two legs on floor)
        self._frame_stand_phase = 8
        # frame number for 2-nd phase of gait (one leg in air)                  
        self._frame_swing_phase = 12                
        
        self.right_first = True
        self._init_poses = 20

        self.walk_step = 0
        self.lateral_step = 0
        self.walk_turn = 0
        self.cycle = 0
        self.cycles_num = 0
        self.right_foot_target_point = Vector(0, -0.0534, -self.gait_height)
        self.right_foot_orientation = Quaternion(0, 0, -1, 0)

        self.left_foot_target_point = Vector(0, 0.0534, -self.gait_height)
        self.left_foot_orientation = Quaternion(0, 0, -1, 0)

    def update(self, walk_step, lateral_step, walk_turn, cycle, cycles_num):
        self.walk_step = walk_step
        self.lateral_step = lateral_step
        self.walk_turn = walk_turn
        self.cycle = cycle
        self.cycles_num = cycles_num

    def compute_angles(self, right_foot_target_point, right_foot_orientation,
    left_foot_target_point, left_foot_orientation):
        data = {}
        right_leg_angles=[]
        left_leg_angles=[]

        # compute inversed kinematics 
        right_leg_solutions = compute_leg_ik(right_foot_target_point, 
                                right_foot_orientation, self._model)
        left_leg_solutions = compute_leg_ik(left_foot_target_point, 
                                left_foot_orientation, self._model)

        #print('time elapsed in compute_Alpha:', clock1.avg())

        if len(right_leg_solutions) == 2:
            if right_leg_solutions[0]['8'] < right_leg_solutions[1]['8']: 
                right_leg_angles = right_leg_solutions[0]
            else: 
                right_leg_angles = right_leg_solutions[1] 
        elif len(right_leg_solutions) == 1:
            right_leg_angles = right_leg_solutions[0]     
        else:
            return {}

        if len(left_leg_solutions) == 2:
            if left_leg_solutions[0]['8'] < left_leg_solutions[1]['8']: 
                left_leg_angles = left_leg_solutions[0]
            else: 
                left_leg_angles = left_leg_solutions[1] 
        elif len(left_leg_solutions) == 1:
            left_leg_angles = left_leg_solutions[0]     
        else:
            return {}

        if self.right_first == True:
            for servo in right_leg_angles:
                data['right_'+servo] = right_leg_angles[servo]
            data['torso'] = 0.0
            for servo in left_leg_angles:
                data['left_'+servo] = -left_leg_angles[servo]
            if self.arms_enabled:
                data['right_elbow_pitch'] = -1.745
                data['right_elbow_yaw'] = 0.0
                data['right_shoulder_roll'] = 0.0
                data['right_shoulder_pitch'] = 0.524 - self.left_foot_target_point.x / 0.0573
                data['left_elbow_pitch'] = 1.745
                data['left_elbow_yaw'] = 0.0
                data['left_shoulder_roll'] = 0.0
                data['left_shoulder_pitch'] = -0.524 + self.right_foot_target_point.x / 0.0573
            else:
                data['right_elbow_pitch'] = 0.0
                data['right_elbow_yaw'] = 0.0
                data['right_shoulder_roll'] = 0.0
                data['right_shoulder_pitch'] = 0.0
                data['left_elbow_pitch'] = 0.0
                data['left_elbow_yaw'] = 0.0
                data['left_shoulder_roll'] = 0.0
                data['left_shoulder_pitch'] = 0.0
        else:
            for servo in left_leg_angles:
                data['left_'+servo] = left_leg_angles[servo]
            data['torso'] = 0.0                                  # Tors
            for servo in left_leg_angles:
                data['right_'+servo] = -right_leg_angles[servo]
            if self.arms_enabled:
                data['right_elbow_pitch'] = -1.745
                data['right_elbow_yaw'] = 0.0
                data['right_shoulder_roll'] = 0.0
                data['right_shoulder_pitch'] = 0.524 - self.right_foot_target_point.x / 0.0573
                data['left_elbow_pitch'] = 1.745
                data['left_elbow_yaw'] = 0.0
                data['left_shoulder_roll'] = 0.0
                data['left_shoulder_pitch'] = -0.524 + self.left_foot_target_point.x / 0.0573
            else:
                data['right_elbow_pitch'] = 0.0
                data['right_elbow_yaw'] = 0.0
                data['right_shoulder_roll'] = 0.0
                data['right_shoulder_pitch'] = 0.0
                data['left_elbow_pitch'] = 0.0
                data['left_elbow_yaw'] = 0.0
                data['left_shoulder_roll'] = 0.0
                data['left_shoulder_pitch'] = 0.0

        return data

    def enter(self):
        #tmp1 = self.right_first
        #if self.walk_turn>0 or self.lateral_step>0:  self.right_first = False
        #else: self.right_first = True
        data = []
        for i in range(self._init_poses):
            self.right_foot_target_point.z = -0.223 + i * (0.223 - self.gait_height) / self._init_poses
            self.left_foot_target_point.z = -0.223 + i * (0.223 - self.gait_height) / self._init_poses
            self.right_foot_target_point.y = -0.0534 - i * self.amplitude / 2 / self._init_poses
            self.left_foot_target_point.y =  0.0534 - i * self.amplitude / 2 / self._init_poses
            leg_angles_frame = self.compute_angles(
                self.right_foot_target_point, 
                self.right_foot_orientation,
                self.left_foot_target_point, 
                self.left_foot_orientation)
            data.append(leg_angles_frame)
        return data

    def tick(self):
        data = []
        #tmp1 = self.right_first
        #if walk_turn>0 or lateral_step>0:  self.right_first = False
        #else: self.right_first = True
        walk_turn = -self.walk_turn / 200
        alpha = 0
        alpha01 = math.pi / self._frame_stand_phase * 2
        frames_per_cycle = 2 * self._frame_stand_phase + 2 * self._frame_swing_phase
        for frame in range(0, frames_per_cycle, 2):
            if 0 <= frame < self._frame_stand_phase:
                alpha = alpha01 * (frame / 2 + 1)
                S = (self.amplitude / 2 + self.lateral_step / 2) * math.cos(alpha)
                self.right_foot_target_point.y = S - 0.0534 + self.lateral_step / 2
                self.left_foot_target_point.y = S + 0.0534 + self.lateral_step / 2
                self.left_foot_target_point.z = -self.gait_height
                self.right_foot_target_point.z = -self.gait_height
                if self.cycle == 0: 
                    continue
                    #dx0 = 0
                else: 
                    dx0 = self.walk_step / \
                        (2 * self._frame_stand_phase + self._frame_swing_phase + 4) * 2
                #self.left_foot_target_point.x = self.walk_step/4 - dx0*(i/2+1)
                xtl0 = -(self._frame_swing_phase + self._frame_stand_phase + 4) * \
                    dx0 / 2 + self.walk_step
                xtr0 = self.walk_step / 2 - \
                    (self._frame_swing_phase + self._frame_stand_phase + 4) * dx0 / 2
                self.left_foot_target_point.x = xtl0 - dx0 * (frame / 2 + 1)
                self.right_foot_target_point.x = xtr0 - dx0 * (frame / 2 + 1)
            if self._frame_stand_phase + self._frame_swing_phase <= \
            frame < 2 * self._frame_stand_phase + self._frame_swing_phase:
                alpha = alpha01 * ((frame - self._frame_swing_phase) / 2 + 1)
                S = (self.amplitude / 2 + self.lateral_step / 2) * math.cos(alpha)
                self.right_foot_target_point.y = S - 0.0534 - self.lateral_step / 2
                self.left_foot_target_point.y = S + -0.0534 + self.lateral_step / 2
                self.left_foot_target_point.z = -self.gait_height
                self.right_foot_target_point.z = -self.gait_height
                dx0 = self.walk_step / \
                    (2 * self._frame_stand_phase + self._frame_swing_phase + 4) * 2  
                self.left_foot_target_point.x = self.left_foot_target_point.x - dx0
                self.right_foot_target_point.x = self.right_foot_target_point.x - dx0
            if self._frame_stand_phase <= frame < self._frame_stand_phase + self._frame_swing_phase:
                self.right_foot_target_point.z = -self.gait_height + self.step_height
                #self.right_foot_target_point.y = S - 64
                if self.cycle == 0:
                    dx = self.walk_step / 2 / self._frame_swing_phase * 2 
                    dx0 = self.walk_step / \
                        (2 * self._frame_stand_phase + self._frame_swing_phase + 4) * 2  
                    dy = self.lateral_step / self._frame_swing_phase * 2  
                    dy0 = self.lateral_step / \
                        (2 * self._frame_stand_phase + self._frame_swing_phase + 4) * 2  
                else:
                    dx = self.walk_step / self._frame_swing_phase * 2  
                    dx0 = self.walk_step / \
                        (2 * self._frame_stand_phase + self._frame_swing_phase + 4) * 2  
                    dy = self.lateral_step / self._frame_swing_phase * 2  
                    dy0 = self.lateral_step / \
                        (2 * self._frame_stand_phase + self._frame_swing_phase + 4) * 2  
                if frame == self._frame_stand_phase:
                    self.right_foot_target_point.x = self.right_foot_target_point.x - dx0
                    self.right_foot_target_point.y = S - 0.064 + dy0
                elif frame == (self._frame_stand_phase + self._frame_swing_phase - 2):
                    self.right_foot_target_point.x = self.right_foot_target_point.x - dx0
                    self.right_foot_target_point.y = S - 0.064 + 2 * dy0 - self.lateral_step
                else:
                    self.right_foot_target_point.x = self.right_foot_target_point.x + \
                        dx * self._frame_swing_phase / (self._frame_swing_phase - 4)
                    self.right_foot_target_point.y = S - 0.064 + dy0 - dy * \
                        self._frame_swing_phase / (self._frame_swing_phase - 4) * \
                        ((frame - self._frame_stand_phase) / 2)
                    self.left_foot_turn = walk_turn - (frame - self._frame_stand_phase) * \
                        walk_turn / (self._frame_swing_phase - 4) * 2
                    self.right_foot_orientation.w = walk_turn - (frame - self._frame_stand_phase) * \
                        walk_turn / (self._frame_swing_phase - 4) * 2
                self.left_foot_target_point.x = self.left_foot_target_point.x - dx0
                self.left_foot_target_point.y = self.left_foot_target_point.y + dy0
            if 2 * self._frame_stand_phase + self._frame_swing_phase <= frame :
                self.left_foot_target_point.z = -self.gait_height + self.step_height
                #self.left_foot_target_point.y = S + 64
                if self.cycle == self.cycles_num - 1:
                    dx0 = self.walk_step / \
                        (2 * self._frame_stand_phase + self._frame_swing_phase + 4) * 4 / \
                        self._frame_swing_phase * 2     # 8.75/6
                    dx = (self.walk_step * (self._frame_stand_phase + self._frame_swing_phase) / \
                        (4 * self._frame_stand_phase) + 2 * dx0) / (self._frame_swing_phase - 4) * 2
                    if frame == (2 * self._frame_stand_phase + 2 * self._frame_swing_phase - 2):
                        self.left_foot_target_point.z = -self.gait_height
                        self.left_foot_target_point.y = S + 0.0534
                else:
                    dx = self.walk_step / (self._frame_swing_phase - 4) * 2
                    dx0 = self.walk_step / \
                        (2 * self._frame_stand_phase + self._frame_swing_phase + 4) * 2
                    dy = self.lateral_step / (self._frame_swing_phase - 4) * 2  
                    dy0 = self.lateral_step / \
                        (2 * self._frame_stand_phase + self._frame_swing_phase + 4) * 2  
                if frame == (2 * self._frame_stand_phase + self._frame_swing_phase):
                    self.left_foot_target_point.x = self.left_foot_target_point.x - dx0
                    self.left_foot_target_point.y = S + 0.064 + dy0
                elif frame == (2 * self._frame_stand_phase + 2 * self._frame_swing_phase - 2):
                    self.left_foot_target_point.x = self.left_foot_target_point.x - dx0
                    self.left_foot_target_point.y = S + 0.064 + 2 * dy0 - self.lateral_step
                else:
                    self.left_foot_target_point.x = self.left_foot_target_point.x + dx
                    self.left_foot_target_point.y = S + 0.064 + dy0 - dy * \
                        (frame -(2 * self._frame_stand_phase + self._frame_swing_phase)) / 2
                    self.left_foot_turn = self.right_foot_orientation.w = (frame - \
                        (2 * self._frame_stand_phase + self._frame_swing_phase)) * \
                        walk_turn / (self._frame_swing_phase - 4) * 2 - walk_turn
                self.right_foot_target_point.x = self.right_foot_target_point.x - dx0
                self.right_foot_target_point.y = self.right_foot_target_point.y + dy0

            leg_angles_frame = self.compute_angles(
                self.right_foot_target_point, 
                self.right_foot_orientation, 
                self.left_foot_target_point, 
                self.left_foot_orientation)
            data.append(leg_angles_frame)
        return data

    def exit(self):
        data = []
        for i in range(self._init_poses):
            self.right_foot_target_point.z = -self.gait_height - (i + 1) * \
                (0.223 - self.gait_height) / self._init_poses
            self.left_foot_target_point.z = -self.gait_height - (i + 1) * \
                (0.223 - self.gait_height) / self._init_poses
            self.right_foot_target_point.y = -0.0534 - (self._init_poses - (i + 1)) * \
                self.amplitude / 2 / self._init_poses
            self.left_foot_target_point.y =  0.0534 - (self._init_poses - (i + 1)) * \
                self.amplitude / 2 / self._init_poses
            leg_angles_frame = self.compute_angles(
                self.right_foot_target_point, 
                self.right_foot_orientation,
                self.left_foot_target_point, 
                self.left_foot_orientation)
            data.append(leg_angles_frame)
        return data
            