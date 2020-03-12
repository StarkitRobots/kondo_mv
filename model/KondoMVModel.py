import sys
import json
from model import Model
sys.path.append('model')
from ServoDict import ServoDict

class KondoMVModel(Model):
    def __init__(self):
        super().__init__()
        with open("model/kal.json") as f:
            self.servos = ServoDict(json.loads(f.read()))
        
        self.sizes = {
        "a5": 0.215,  # m distance from symmetry axis to servo 5
        "b5": 0.185,  # м расстояние от оси сервы 5 до оси сервы 6 по горизонтали
        "c5": 0,     # м расстояние от оси сервы 6 до нуля Z по вертикали
        "a6": 0.42,    # м расстояние от оси сервы 6 до оси сервы 7
        "a7": 0.655,  # м расстояние от оси сервы 7 до оси сервы 8
        "a8": 0.638,  # м расстояние от оси сервы 8 до оси сервы 9
        "a9": 0.355,  # м расстояние от оси сервы 9 до оси сервы 10
        "a10": 0.254,  # м расстояние от оси сервы 10 до центра стопы по горизонтали
        "b10": 0.164,  # м расстояние от оси сервы 10 до низа стопы
        "c10": 0.12   # м расстояние от оси сервы 6 до оси сервы 10 по горизонтали
        }

if __name__ == "__main__":
    kondo = KondoMVModel()
    print(kondo.servos['torso'])
