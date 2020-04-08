import json
import copy

class MoveReader:
    """Class reads data from specifically formatted json file and converts
    it to data for servos.

    JSON file format:
    {
        'enter':
        {
            '1': 
            {
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
                "servo1_name": servo1_pos_in_deg,
                "servo2_name": servo2_pos_in_deg,
                ...
            }
            ...
        }
    }
    """

    def __init__(self, json_file_name):
        """Class initilization
        
        Arguments:
            json_file_name {str} -- json file name of the move
        """
        self.move_config = json.loads(open(json_file_name).read())
        self.move_state = ''
        self.current_pose = ''
        self.next_pose = ''
        self.frame_num = 20
        self.frames_list = []

    
    def read_next_pose(self):
        """Reads next pose from json file according to the state and current_pose
        
        Returns:
            pose {dict} -- pose with servos to change. Format of pose: {"servo_name": servo_pos_in_rad}
        """
        pose = {}
        return pose

    def calculate_frame_list(self, current_pose=None, next_pose=None):
        """Takes 2 poses and divide all servos' shifts into 'self.frame_num' parts
        
        Keyword Arguments:
            current_pose {dict} -- written in json file current pose (default: self.current_pose)
            next_pose {dict} -- written in json file next pose you target (default: self.next_pose)
        
        Returns:
            frames_list [list] -- list of frames. Each frame is a dict of servos and its positions
        """
        current_pose = self.current_pose if current_pose is None else copy.deepcopy(current_pose)
             
        next_pose = self.next_pose if next_pose is None else copy.deepcopy(next_pose)
            


        return self.frames_list

    def get_frame(self):
        """Returns a frame to process for MoveScheduler.
        
        Returns:
           frame [dict] -- dict of servos and its positions
        """
        frame = {}
        return frame