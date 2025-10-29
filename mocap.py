import threading
import os
import json
import time
import logging
import motioncapture

from datetime import datetime
from log import LoggerFactory


# The host name or ip address of the mocap system
host_name = '192.168.1.39'

# The type of the mocap system
# Valid options are: 'vicon', 'optitrack', 'optitrack_closed_source', 'qualisys', 'nokov', 'vrpn', 'motionanalysis'
mocap_system_type = 'vicon'


class MocapWrapper(threading.Thread):
    def __init__(self, body_name, rate, log_level=logging.INFO):
        threading.Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self.set_origin = None
        self._stay_open = True
        self.all_frames = []
        self.position_log = []
        self.rate = rate
        self.logger = LoggerFactory("Mocap", level=log_level).get_logger()

        self.start()

    def close(self):
        self._stay_open = False

        now = datetime.now()
        formatted = now.strftime("%H_%M_%S_%m_%d_%Y")
        file_path = os.path.join("logs", f"{mocap_system_type}_{formatted}.json")
        with open(file_path, "w") as f:
            json.dump({"frames": self.all_frames}, f)
        self.logger.info(f"Mocap log saved in {file_path}")
        self.join()

    def run(self):
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        i = 0
        last_time = time.time()
        while self._stay_open:
            time.sleep(1/self.rate)
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    now = time.time()
                    pos = obj.position
                    if self.on_pose:
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation, now])
                    self.all_frames.append({
                        "frame_id": i,
                        "tvec": [float(pos[0]), float(pos[1]), float(pos[2])],
                        "qvec": [obj.rotation.x, obj.rotation.y, obj.rotation.z, obj.rotation.w],
                        "time": now * 1000
                    })
                    i += 1
            current_time = time.time()
            if (current_time - last_time) > 1:
                current_time_us = int(current_time * 1.0e6)
                self.set_origin(current_time_us)

