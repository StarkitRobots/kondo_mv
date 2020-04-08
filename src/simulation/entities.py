import math
from . import vrep
from . import vrep_consts


class Entity:
    def __init__(self, client_id, sim_name):
        self.client_id = client_id
        _, self._model = vrep.simxGetObjectHandle(self.client_id, 
                                                    sim_name, 
                                                    vrep_consts.simx_opmode_blocking)

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


class Ball(Entity):
    def __init__(self, client_id):
        super().__init__(client_id, 'Ball')


class Robot(Entity):
    def __init__(self, client_id):
        super().__init__(client_id, 'Dummy_H')
        self._vision_sensor = Entity(client_id, 'Vision_sensor') 
    
    def get_camera_orientation_quaternion(self, field):
        return self._vision_sensor.get_orientation_quaternion(field)

    def get_camera_orientation_euler(self, field):
        return self._vision_sensor.get_orientation_euler(field)

    def get_camera_data_img(self):
        _, _, image_data = vrep.simxGetVisionSensorImage(self.client_id, 
                                                        self._vision_sensor, 
                                                        0,
                                                        vrep_consts.simx_opmode_streaming)
        return image_data


class Field(Entity):
    def __init__(self, client_id):
        super().__init__(client_id, 'ResizableFloor_5_25')