from moves.move import Move

class MoveScheduler:
    def __init__(self, model):
        self.active_moves = []
        self._model = model
        self.servos = {}
        self.servos_zero = {}
        self.status = ''
        self.kondo_motions_to_apply = []
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
        servos = {}
        for servo in self.servos:
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
            for servo in servos.values():
                servo = None
        return new_frames_list

    def _combine_frames(self, frames):
        servos = self.servos
        for servo in servos.values():
            servo = None
        for servo in servos:
            for frame in frames:
                try:
                    if servo in frame:
                        if servos[servo] is None:
                            servos[servo] = 0
                        servos[servo] += frame[servo]
                except IndexError:
                    pass
        new_frame = {}
        for servo in servos:
            if servos[servo] is not None:
                new_frame[servo] = servos[servo]
        return new_frame
        
    def get_kondo_motion(self):
        if self.kondo_motions_to_apply != []:
            return self.kondo_motions_to_apply.pop(0)
        else:
            return {}

    def start_move(self, move):
        for active_move in self.active_moves:
            if type(move) == type(active_move):
                return
        # isinstance is very sensitive to module name so import influences too much on it
        #if isinstance(move, Move):
        self.active_moves.append(move)
        move.start()

    def stop_move(self, move):
        if move in self.active_moves:
            move.stop()

    def tick(self):
        if self.active_moves != []:
            print('active moves not empty')
            frames = []
            if any([move.is_kondo_motion for move in self.active_moves]):
                print('any kondo motion')
                for move in self.active_moves:
                    if not move.is_kondo_motion:
                        self.stop_move(move)
            
            if all([move.is_kondo_motion for move in self.active_moves]):
                print('all kondo')
                self.kondo_motions_to_apply = [move.tick() for move in self.active_moves]
            if self.kondo_motions_to_apply == []:
                print('frames mode')
                for move in self.active_moves:
                    print(move.frames_to_process)
                    if move.frames_to_process != []:
                        print('frames not empty')
                        frames.append(move.get_frame())
                frame_to_update = self._combine_frames(frames)
                frame_to_update = self.add_zeros(frame_to_update)
                self.update_servos(frame_to_update)
            for move in self.active_moves:
                if move.enabled == False and move.frames_to_process == []:
                    self.active_moves.pop(self.active_moves.index(move))
            
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
    walk.enabled = True
    head.enabled = True
    walk.update(0, 0.05, 0, 1, 2)
    ms.start_move(walk)
    ms.start_move(head)
    while True:
    #while walk.frames_to_process != []:
        print(len(walk.frames_to_process))
        print(ms.servos)
        ms.tick()
