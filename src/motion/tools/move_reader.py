import json
import copy
from math import fabs
import collections

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
        self.move_config = json.loads(open("standup_front.json").read())  #slovar 
        self.move_state = ''   #читаем какое  именно состояние 
        self.current_pose = ''  #берем значение ключа 
        self.next_pose = ''     #берем значение ключа 
        self.frame_num = 20
        self.frames_list = []  
       
      


  
            

    
    def read_next_pose(self):   """мы знаем какой сейчас тик""" 
        
        
        if self.current_pose  == collections.deque(self.move_config, maxlen=1):
            print(" is last pose")
        else:
            rerurn self.next_pose = self.current_pose.index() + 1
        
        """Reads next pose from json file according to the state and current_pose
        
        Returns:
            pose {dict} -- pose with servos to change. Format of pose: {"servo_name": servo_pos_in_rad}
        """
        pose = {}
        return pose

    def count_interm_andle(self, current_pose,next_pose, frames_list)
        retern fabs(self.current_pose.get(key, default=None) - self.next_pose.get(key, default=None))/self.frames_num


    def  exception_read_next_pose()
        if self.next_pose:
            print("we have next_pose")
        else:
            return read_next_pose()
       

    def calculate_frame_list(self, current_pose=None, next_pose=None):
      
        exception_read_next_pose()

        self.current_angle = self.current_pose.get(key, default=None)     
        self.interm_andle  = count_interm_andle()

        for self.interm_angle in range(self.current_pose.get(key, default=None),self.next_pose.get(key, default=None))
            self.interm_pose = self.current_pose.get(key, default=None) + self.interm_andle
            self.frames_list.update(self.interm_pose)
        
        


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

    
    
    
    def get_frame(self): #тут получаем список фреймов и делаем  гет фрайм  . смотрит на список и он пустой вызывае колкулейт фрейм лист берет нынешнюю позу тк должна инициализирована в ней мы должны сделать проверку если  есть след позда то будем ее , возвращаем в гет фрейм и получает поп 
       
        if self.frames_list:
            print("frame_list has data")
            return self.frames_list.items()

        else:
            print("Frame_list empty")
            return calculate_frame_list(self, current_pose, next_pose) 
       

       
        """Returns a frame to process for MoveScheduler.
        



        Returns:
           frame [dict] -- dict of servos and its positions
        """
        frame = {}
        return frame




    
