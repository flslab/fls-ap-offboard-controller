import time


class VelocityEstimator:
    def __init__(self, filter_alpha=0.3):
        self.prev_position = None
        self.prev_time = None
        self.velocity = [0.0, 0.0, 0.0]
        self.filter_alpha = filter_alpha  # Low-pass filter coefficient

    def update(self, x, y, z, timestamp=None):
        if timestamp is None:
            timestamp = time.time()

        if self.prev_position is not None and self.prev_time is not None:
            dt = timestamp - self.prev_time

            if dt > 0:
                # Calculate raw velocity
                vx_raw = (x - self.prev_position[0]) / dt
                vy_raw = (y - self.prev_position[1]) / dt
                vz_raw = (z - self.prev_position[2]) / dt

                # Apply low-pass filter to smooth velocity
                self.velocity[0] = (1 - self.filter_alpha) * self.velocity[0] + self.filter_alpha * vx_raw
                self.velocity[1] = (1 - self.filter_alpha) * self.velocity[1] + self.filter_alpha * vy_raw
                self.velocity[2] = (1 - self.filter_alpha) * self.velocity[2] + self.filter_alpha * vz_raw

        self.prev_position = [x, y, z]
        self.prev_time = timestamp

        return self.velocity
