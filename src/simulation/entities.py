import math
import cv2.cv2 as cv2
import numpy as np
from . import vrep
from . import vrep_consts


class Entity:
    def __init__(self, client_id, sim_handle_name, euler=True):
        self.client_id = client_id
        _, self._model = vrep.simxGetObjectHandle(self.client_id, 
                                                    sim_handle_name, 
                                                    vrep_consts.simx_opmode_blocking)
        self._use_euler = euler
        self.position = None
        self.orientation = None

    def get_position(self, relative_object):
        if relative_object.__hasattr__('_model'):
            _, pos = vrep.simxGetObjectPosition(self.client_id, 
                                                self._model,
                                                relative_object._model, 
                                                vrep_consts.simx_opmode_streaming)
        else: 
            _, pos = vrep.simxGetObjectPosition(self.client_id, 
                                                self._model,
                                                relative_object, 
                                                vrep_consts.simx_opmode_streaming)
        return pos

    def get_orientation_quaternion(self, relative_object):
        if relative_object.__hasattr__('_model'):
            _, orn = vrep.simxGetObjectQuaternion(self.client_id, 
                                                    self._model,
                                                    relative_object._model, 
                                                    vrep_consts.simx_opmode_streaming)
        else:
            _, orn = vrep.simxGetObjectQuaternion(self.client_id, 
                                                self._model,
                                                relative_object, 
                                                vrep_consts.simx_opmode_streaming)
        return orn

    def get_orientation_euler(self, relative_object):
        if relative_object.__hasattr__('_model'):
            _, orn = vrep.simxGetObjectOrientation(self.client_id, 
                                                    self._model,
                                                    relative_object._model, 
                                                    vrep_consts.simx_opmode_streaming)
        else:
            _, orn = vrep.simxGetObjectOrientation(self.client_id, 
                                                self._model,
                                                relative_object, 
                                                vrep_consts.simx_opmode_streaming)
        return orn

    def set_position(self, new_position, relative_object):
        if relative_object.__hasattr__('_model'):
            vrep.simxSetObjectPosition(self.client_id, 
                                            self._model,
                                            relative_object._model,
                                            new_position, 
                                            vrep_consts.simx_opmode_streaming)
        else:
            vrep.simxSetObjectPosition(self.client_id, 
                                            self._model,
                                            relative_object,
                                            new_position, 
                                            vrep_consts.simx_opmode_streaming)

    def set_orientation_quaternion(self, new_quaternion, relative_object):
        if relative_object.__hasattr__('_model'):
            vrep.simxSetObjectQuaternion(self.client_id, 
                                            self._model,
                                            relative_object._model,
                                            new_quaternion, 
                                            vrep_consts.simx_opmode_streaming)
        else:
            vrep.simxSetObjectQuaternion(self.client_id, 
                                            self._model,
                                            relative_object,
                                            new_quaternion, 
                                            vrep_consts.simx_opmode_streaming)

    def set_orientation_euler(self, new_euler, relative_object):
        if relative_object.__hasattr__('_model'):
            vrep.simxSetObjectOrientation(self.client_id, 
                                            self._model,
                                            relative_object._model,
                                            new_euler, 
                                            vrep_consts.simx_opmode_streaming)
        else:
            vrep.simxSetObjectOrientation(self.client_id, 
                                            self._model,
                                            relative_object,
                                            new_euler, 
                                            vrep_consts.simx_opmode_streaming)

    def update(self, relative_object):
        self.position = self.get_position(relative_object)
        if self._use_euler:
            self.orientation = self.get_orientation_euler(relative_object)
        else:
            self.orientation = self.get_orientation_quaternion(relative_object)

class Ball(Entity):
    def __init__(self, client_id):
        super().__init__(client_id, 'Ball')


class Robot(Entity):
    def __init__(self, client_id):
        super().__init__(client_id, 'Dummy_H')
        self._vision_sensor = Entity(client_id, 'Vision_sensor') 
        self.show_fake_vision = False
        self.sensor_frame = None
    
    def get_camera_orientation_quaternion(self, field):
        return self._vision_sensor.get_orientation_quaternion(field)

    def get_camera_orientation_euler(self, field):
        return self._vision_sensor.get_orientation_euler(field)

    def get_camera_image(self):
        _, resolution, image_data = vrep.simxGetVisionSensorImage(self.client_id, 
                                                        self._vision_sensor, 
                                                        0,
                                                        vrep_consts.simx_opmode_streaming)
        nuimg = np.array(image_data, dtype=np.uint8)
        nuimg.shape = (resolution[1], resolution[0], 3)
        nuimg1 = cv2.cvtColor(nuimg, cv2.COLOR_RGB2BGR)
        img = np.flip(nuimg1, 1)
        return img

    def display(self):
        cv2.imshow('Fake sensor', self.sensor_frame)
        cv2.waitKey(10) & 0xFF
    
    def update(self, relative_object):
        super().update(relative_object)
        self.sensor_frame = self.get_camera_image()
        if self.show_fake_vision:
            self.display


class Field(Entity):
    def __init__(self, client_id):
        super().__init__(client_id, 'ResizableFloor_5_25')