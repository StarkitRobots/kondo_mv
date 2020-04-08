import json

class ControllerManager:
    def __init__(self, simulation=False, *args):    
        with open("model/kal.json", "r") as f:
            self.servos = json.loads(f.read())
        try:
            if simulation:
                from ..simulation import SimulationController
                self.controller = SimulationController(self.servos, *args)
        except KeyError:
            from ..lowlevel import KondoController
            self.controller = KondoController()
        print('ControllerManager: Controller acknowledge ', self.check_acknowledge())

    def check_acknowledge(self):
        return self.controller.check_acknowledge()

    def set_servos(self, servo_data):
        self.controller.set_servos(servo_data)

    def falling_test(self):
        self.controller.falling_test()