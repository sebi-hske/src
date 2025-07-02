class MotionPredictor:
    def __init__(self):
        self.position_history = {}  # {id: [(x_mm, timestamp), ...]}
        self.prediction_given = set()

    def update(self, obj_id, x_mm, timestamp, in_speed_zone):
        if obj_id not in self.position_history:
            self.position_history[obj_id] = []

        if in_speed_zone:
            self.position_history[obj_id].append((x_mm, timestamp))
            self.position_history[obj_id] = [
                (x, t) for (x, t) in self.position_history[obj_id]
                if timestamp - t <= 10.0
            ]

    def compute_velocity_from_first_last(self, obj_id):
        positions = self.position_history.get(obj_id, [])
        if len(positions) < 2:
            return None
        x0, t0 = positions[0]
        x1, t1 = positions[-1]
        if t1 - t0 == 0:
            return None
        return (x1 - x0) / (t1 - t0)

    def predict_arrival_time(self, obj_id, current_time, target_x_mm):
        if obj_id not in self.position_history or not self.position_history[obj_id]:
            return None
        last_x, _ = self.position_history[obj_id][-1]
        vx = self.compute_velocity_from_first_last(obj_id)
        if vx is None or vx <= 0:
            return None
        dt = (target_x_mm - last_x) / vx
        if dt < 0:
            return None
        return current_time + dt