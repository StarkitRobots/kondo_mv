from lowlevel.kondo_controller import Rcb4BaseLib
from pyb import UART
class Motion:
    def __init__(self):
        kondo = Rcb4BaseLib()
        

    def apply(self, action):
        return 0