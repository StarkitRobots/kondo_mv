from moves.move import Move

class MoveScheduler:
    def __init__(self, model):
        self.active_moves = []
        self._model = model
        self.servos = {}
        self.servos_zero = {}
        self.status = ''
        self._frames_to_process = None
        for servo in self._model.servos:
            self.servos[servo] = self.servos_zero[servo] = model.servos[servo]['zero']

    def _is_motion(self, data):
        if data is dict:
            if 'motion' in data.keys:
                return True
            else:
                return False
        else:
            return False

    def add_zeros(self, frame):
        for servo in frame:
            frame[servo] += self.servos_zero[servo]
        return frame

    def combine_moves(self, *args):
        new_move = Move()
        moves = []
        args_without_moves = list(args)
        for arg in args_without_moves:
            if isinstance(arg, Move):
                moves.append(arg)
                args_without_moves.pop(arg)
        new_move.enter = \
            lambda *args_without_moves: \
                self._combine_frames_lists([move.enter(*args_without_moves) for move in moves])
        new_move.tick = \
            lambda *args_without_moves: \
                self._combine_frames_lists([move.tick(*args_without_moves) for move in moves])
        new_move.exit = \
            lambda *args_without_moves: \
                self._combine_frames_lists([move.exit(*args_without_moves) for move in moves])
        new_move.name = sum([move.name + '+' for move in moves])
        new_move.name = new_move.name[:len(new_move.name) - 1]
        return new_move

    def _combine_frames_lists(self, frames_lists):
        servos = self.servos
        for servo in servos:
            servos[servo] = None
        new_frames_list = []
        max_frames_num = max([len(frames_list) for frames_list in frames_lists])
        for i in range(max_frames_num):
            for frames_list in frames_lists:
                for servo in servos:
                    try:
                        if servo in frames_list[i]:
                            if servos[servo] is None:
                                servos[servo] = 0
                            servos[servo] += frames_list[i][servo]
                    except IndexError:
                        pass
                active_servos = {}
                for servo in servos:
                    if servos[servo] is not None:
                        active_servos[servo] = servos[servo]
            new_frames_list.append(active_servos)
            for servo in servos:
                servos[servo] = None
        return new_frames_list

    def start_move(self, move):
        for active_move in self.active_moves:
            if type(move) == type(active_move):
                return []

        if move is not None:
            self.active_moves.append(move)
            frames_to_process = move.enter()
            if frames_to_process == [] or frames_to_process == {} or frames_to_process is None:
                return []
            else:
                self._frames_to_process = frames_to_process

    def stop_move(self, move):
        if move in self.active_moves:
            self.active_moves.pop(move)
            frames_to_process = move.exit()
        if frames_to_process == [] or frames_to_process == {} or frames_to_process is None:
            return []
        else:
            self._frames_to_process = frames_to_process

    def tick(self):
        if self._frames_to_process is None:
            self._frames_to_process = self.current_move.tick(*args)
        
        if self._is_motion(self._frames_to_process):
            return self._frames_to_process
        else:
            frame = self._frames_to_process[0]
            self._frames_to_process.pop(0)
            frame = self.add_zeros(frame)
            self.update_servos(frame)
            if len(self._frames_to_process) == 0:
                self._frames_to_process = None
            return {}
            
    def update_servos(self, frame):
        for servo in frame:
            self.servos[servo] = frame[servo]

if __name__ == "__main__":
    import sys
    sys.path.append('model')
    sys.path.append('model/utils')
    sys.path.append('motion/moves')
    from KondoMVModel import KondoMVModel
    from moves.head import Head
    from moves.walk import Walk
    head = Head()
    model = KondoMVModel()
    walk = Walk(True, model)
    ms = MoveScheduler(model)
    head_frames = head.tick()
    walk.enabled = True
    walk_frames = walk.tick(0, 0.05, 0, 1, 2)
    ms.start_move(walk, head)
    print(ms.tick(0, 0.05, 0, 1, 2))
    #print(walk_frames)
