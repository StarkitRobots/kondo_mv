from .moves import Move

class MoveScheduler:
    def __init__(self, model):
        self.active_moves = []
        self._model = model
        self.servos = {}
        self.servos_zero = {}
        self.status = ''
        for servo in self._model.servos:
            self.servos[servo] = self.servos_zero[servo] = model.servos[servo]['zero']

    def has_active_move(self, move_name):
        return any([move_name == move.name for move in self.active_moves])
            
    def add_zeros(self, frame):
        for servo in frame:
            frame[servo] += self.servos_zero[servo]
        return frame

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
            for servo in servos:
                servo = None
        return new_frames_list

    def _combine_frames(self, frames):
        servos = {}
        for servo in self.servos:
            servos[servo] = None
        for servo in servos:
            for frame in frames:
                try:
                    if servo in frame:
                        if servos[servo] is None:
                            servos[servo] = 0.0
                        servos[servo] += frame[servo]
                except IndexError:
                    pass
        new_frame = {}
        for servo in servos:
            if servos[servo] is not None:
                new_frame[servo] = servos[servo]
        return new_frame

    def start_move(self, move):
        if self.has_active_move(move.name):
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
            frames = []
            for move in self.active_moves:
                if move.frames_to_process != []:
                    frames.append(move.get_frame())
                else:
                    if move.enabled:
                        move.tick()
            
            if frames != []:
                frame_to_update = self._combine_frames(frames)
                
                frame_to_update = self.add_zeros(frame_to_update)
                self.update_servos(frame_to_update)
            
            for move in self.active_moves:
                if not move.enabled and move.frames_to_process == []:
                    self.active_moves.pop(self.active_moves.index(move))
            
    def update_servos(self, frame):
        for servo in frame:
            self.servos[servo] = frame[servo]
