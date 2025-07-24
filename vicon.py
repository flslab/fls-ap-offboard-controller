import json
import logging
import os.path
import threading
import time
from datetime import datetime

from pyvicon_datastream import PyViconDatastream, StreamMode, Direction

from log import LoggerFactory

VICON_PC_IP = '192.168.1.39'
VICON_ADDRESS = f"{VICON_PC_IP}:801"


class ViconWrapper(threading.Thread):
    def __init__(self, callback=None, log_level=logging.INFO, labeled_object=False):
        super().__init__()
        self.running = False
        self.logger = LoggerFactory("Vicon", level=log_level).get_logger()
        self.callback = callback
        self.position_log = []
        self.labeled_object = labeled_object

    def stop(self):
        self.running = False
        self.join()

    def run(self):
        self.running = True

        client = PyViconDatastream()
        self.logger.info("Client object created.")

        try:
            self.logger.info(f"Attempting to connect to Vicon server at {VICON_ADDRESS}...")
            client.connect(VICON_ADDRESS)
            self.logger.info("Connection successful!")

            if self.labeled_object:
                client.enable_marker_data()
                client.enable_segment_data()
                self.logger.info("Marker and Segment data enabled.")
            else:
                client.enable_unlabeled_marker_data()
                self.logger.info("Unlabeled marker data enabled.")

            client.set_stream_mode(StreamMode.ClientPull)
            self.logger.info("Stream mode set to ClientPull.")

            # Set axis mapping for standard coordinate systems
            client.set_axis_mapping(Direction.Forward, Direction.Left, Direction.Up)

            last_time = time.time()
            last_pos_x, last_pos_y, last_pos_z = 0, 0, 0
            while self.running:
                if client.get_frame():
                    frame_num = client.get_frame_number()
                    self.logger.debug(f"--- Frame {frame_num} ---")

                    if self.labeled_object:
                        object_count = client.get_subject_count()
                        self.logger.debug(f"\tSubject count: {object_count}")
                    else:
                        object_count = client.get_unlabeled_marker_count()
                        self.logger.debug(f"\tUnlabeled marker count: {object_count}")

                    if object_count is not None and object_count == 1:
                        translation = None

                        if self.labeled_object:
                            subject_name = client.get_subject_name(0)
                            if subject_name:
                                self.logger.debug(f"\tSubject: {subject_name}")
                                root_segment = client.get_subject_root_segment_name(subject_name)
                                translation = client.get_segment_global_translation(subject_name, root_segment)
                        else:
                            translation = client.get_unlabeled_marker_global_translation(0)

                        if translation is not None:
                            now = time.time()
                            dt = now - last_time
                            pos_x, pos_y, pos_z = translation
                            vel_x, vel_y, vel_z = (pos_x - last_pos_x) / dt, (pos_y - last_pos_y) / dt, (pos_z - last_pos_z) / dt
                            self.position_log.append({
                                "frame_id": frame_num,
                                "tvec": [pos_x, pos_y, pos_z],
                                "vel": [vel_x, vel_y, vel_z],
                                "time": now * 1000
                            })
                            if callable(self.callback):
                                self.callback(pos_x, pos_y, pos_z, vel_x, vel_y, vel_z)
                            last_time = now
                            last_pos_x, last_pos_y, last_pos_z = pos_x, pos_y, pos_z
                            self.logger.debug(
                                f"\tPosition (mm): X={pos_x:.2f}, Y={pos_y:.2f}, Z={pos_z:.2f}")
                        else:
                            self.logger.warning(f"\tPosition (mm): Occluded or no data")

                else:
                    time.sleep(0.01)

        except KeyboardInterrupt:
            self.logger.error("\nScript interrupted by user.")
        except Exception as e:
            self.logger.error(f"An unexpected error occurred: {e}")

        finally:
            now = datetime.now()
            formatted = now.strftime("%H_%M_%S_%m_%d_%Y")
            file_path = os.path.join("logs", f"vicon_{formatted}.json")
            with open(file_path, "w") as f:
                json.dump({"frames": self.position_log}, f)
            self.logger.info(f"Vicon log saved in {file_path}")

            if client.is_connected():
                client.disconnect()
                self.logger.info("Disconnected from Vicon server.")


if __name__ == "__main__":
    vw = ViconWrapper(labeled_object=False, log_level=logging.DEBUG)
    vw.start()
    time.sleep(30)
    vw.stop()

    # July 15
    # E
    # vicon_16_57_16_07_15_2025.json
    # O
    # vicon_17_01_00_07_15_2025.json
    # S
    # vicon_17_51_44_07_15_2025.json
    # N
    # vicon_17_55_20_07_15_2025.json

    # E:
    # logs/16_02_08_06_30_2025/log.json
    # vicon_16_03_42_06_30_2025.json * (4224, 6843)

    # N:
    # logs/16_07_54_06_30_2025/log.json
    # vicon_16_09_30_06_30_2025.json * (3590, 5801)

    # S:
    # logs/16_11_28_06_30_2025/log.json
    # vicon_16_12_16_06_30_2025.json

    # S linear speed=0.25
    # logs/16_16_10_06_30_2025/log.json
    # vicon_16_17_18_06_30_2025.json
    # logs/16_18_04_06_30_2025/log.json
    # vicon_16_36_10_06_30_2025.json * (2862, 3764)

    # S linear speed=0.15
    # logs/16_38_10_06_30_2025/log.json
    # vicon_16_39_22_06_30_2025.json
    # logs/16_42_44_06_30_2025/log.json
    # vicon_16_43_51_06_30_2025.json
    # logs/16_49_57_06_30_2025/log.json
    # vicon_16_51_32_06_30_2025.json

    # O linear speed=0.25:
    # logs/16_13_46_06_30_2025/log.json
    # vicon_16_14_37_06_30_2025.json * (1743, 2483)

    # O linear speed=0.2:
    # 16_53_10_06_30_2025/log.json
    # vicon_16_54_39_06_30_2025.json

    # O linear speed=0.15:
    # 16_56_16_06_30_2025/log.json
    # vicon_16_57_15_06_30_2025.json
