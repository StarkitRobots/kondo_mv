import json
import copy
import time

class MoveReader:
    """Class reads data from specifically formatted json file and converts
    it to data for servos.

    JSON file format:
    {
        'enter':
        {
            '1': 
            {
                "frames_num": subpose_num,
                "servo1_name": servo1_pos_in_deg,
                "servo2_name": servo2_pos_in_deg,
                ...
            }
            ...
        }
        'tick':
        {
            '1': 
            {
                "frames_num": subpose_num,
                "servo1_name": servo1_pos_in_deg,
                "servo2_name": servo2_pos_in_deg,
                ...
            }
            ...
        }
        'exit':
        {
            '1': 
            {
                "frames_num": subpose_num,
                "servo1_name": servo1_pos_in_deg,
                "servo2_name": servo2_pos_in_deg,
                ...
            }
            ...
        }
    }
    It is not necessary to specify all servos. Only servos that change their positions.
    """

    def __init__(self, json_file_name, logger=None):
        """Class initilization
        
        Arguments:
            json_file_name {str} -- json file name of the move
        """
        self.move_config = json.loads(open(json_file_name).read())
        self.move_state = ''
        self.current_pose = ''
        self.next_pose = ''
        self.frames_num = -1
        self.frames_list = []
        self.logger = logger

    def set_move_state(self, move_state):
        self.move_state = move_state


    def get_current_pose(self):
        if not self.move_state:
            raise Exception("Move state is not defined")
        else:
            if not self.current_pose:
                if not len(self.move_config[self.move_state]):
                    return ''
                self.current_pose = '1'
        return self.current_pose
    
    def read_next_pose(self, reversed=False):
        """Reads next pose and number of frames from json file according to the state and current_pose

        Keyword Arguments:
            reversed {bool} -- play move forward or backward(default: False)
        
        Returns:
            frames_num {int} -- number of frames to transform one pose to another
            pose {dict} -- pose with servos to change. Format of pose: {"servo_name": servo_pos_in_rad}
        """
        current_pose = self.get_current_pose()
        if reversed and current_pose == '1':
            print("End of the move_state")
            return 0, ''
        elif not reversed and current_pose == str(len(self.move_config[self.move_state])):
            print("End of the move_state")
            return 0, ''
        elif not current_pose:
            print('move_state is empty')
            return 0, ''

        if not reversed:
            next_pose = str(int(self.current_pose) + 1)
            frames_num = self.move_config[self.move_state][next_pose]['frames_num']
        else:
            next_pose = str(int(self.current_pose) - 1)
            frames_num = self.move_config[self.move_state][current_pose]['frames_num']
        return frames_num, next_pose

    def calculate_frames_list(self, current_pose=None, next_pose=None, frames_num=None):
        """Takes 2 poses and divide all servos' shifts into 'self.frame_num' parts
        
        Keyword Arguments:
            current_pose {str} or {dict} -- written in json file current pose (default: self.current_pose)
            next_pose {str} or {dict} -- written in json file next pose you are targeting (default: self.next_pose)
        
        Returns:
            frames_list [list] -- list of frames. Each frame is a dict of servos and its positions
        """
        if current_pose is None:
            current_pose_dict = self.move_config[self.move_state][self.current_pose]
        else:
            if current_pose is dict:
                current_pose_dict = copy.deepcopy(current_pose)  
            elif current_pose is str:
                current_pose_dict = self.move_config[self.move_state][current_pose]
            else:
                raise Exception("Invalid current_pose type. Only str or dict are supported.")
                
             
        if next_pose is None:
            next_pose_dict = self.move_config[self.move_state][self.next_pose]
        else:
            if next_pose is dict:
                next_pose_dict = copy.deepcopy(next_pose)  
            elif next_pose is str:
                next_pose_dict = self.move_config[self.move_state][next_pose]
            else:
                raise Exception("Invalid next_pose type. Only str or dict are supported.")

        frames_num = self.frames_num if frames_num is None else frames_num

        if not frames_num and not next_pose:
            print("No changes")
            return []


        return self.frames_list

    def get_frame(self, reversed=False):
        """Returns a frame to process for MoveScheduler.

        Keyword Arguments:
            reversed {bool} -- play move forward or backward(default: False)
        
        Returns:
           frame [dict] -- dict of servos and its positions
        """
        if not self.frames_list:
            # take the frame closest to current position  
            frame = self.frames_list.pop(0)
        else:
            # lets calculate next frames_list
            self.current_pose = self.next_pose
            self.frames_num, self.next_pose = self.read_next_pose(reversed=reversed)
            self.frames_list = self.calculate_frames_list()
            frame = self.frames_list.pop(0)

        return frame