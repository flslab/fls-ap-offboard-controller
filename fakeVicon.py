import threading
import os
import json
import time
import logging

# used to fake a quaternion
class pos:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

# send fake vicon data for simulation to see if the drone arms
class fakeVicon(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

        self.send_pos = None
        self.set_origin = None
        self._stay_open = True

        self.start()

    def close(self):
        self._stay_open = False
        self.join()

    def run(self):
        last_time = time.time()
        while self._stay_open:
            current_time = time.time()

            if (current_time - last_time) > 0.01:
                now = time.time()
                pos(1,1,1,1)
                self.send_pos(1,1,1, pos, now)

            if (current_time - last_time) > 1:
                current_time_us = int(current_time * 1.0e6)
                self.set_origin(current_time_us)

