# File: client.py
# Final version fixing the NumPy boolean ambiguity error
import threading
import time
from pyvicon_datastream import PyViconDatastream, StreamMode, Direction

from log import LoggerFactory

# --- Configuration ---
VICON_PC_IP = '192.168.1.39'  # Your Vicon PC's IP address
VICON_ADDRESS = f"{VICON_PC_IP}:801"  # Combine IP and Port into one string


class ViconWrapper(threading.Thread):
    def __init__(self, callback, log_level):
        super().__init__()
        self.running = False
        self.logger = LoggerFactory("Vicon", level=log_level).get_logger()
        self.callback = callback

    def stop(self):
        self.running = False
        self.join()

    def run(self):
        self.running = True
        # Create an instance of the class directly
        client = PyViconDatastream()
        self.logger.info("Client object created.")

        try:
            self.logger.info(f"Attempting to connect to Vicon server at {VICON_ADDRESS}...")
            client.connect(VICON_ADDRESS)
            self.logger.info("Connection successful!")

            # Enable required data types
            client.enable_marker_data()
            client.enable_segment_data()
            self.logger.info("Marker and Segment data enabled.")

            # Set the streaming mode using the imported StreamMode enum
            client.set_stream_mode(StreamMode.ClientPull)
            self.logger.info("Stream mode set to ClientPull.")

            # Set axis mapping for standard coordinate systems (optional but recommended)
            client.set_axis_mapping(Direction.Forward, Direction.Left, Direction.Up)

            # Loop indefinitely until interrupted
            while self.running:
                # Get a frame of data
                if client.get_frame():
                    frame_num = client.get_frame_number()
                    self.logger.debug(f"--- Frame {frame_num} ---")

                    # Get the number of subjects
                    subject_count = client.get_subject_count()
                    if subject_count > 0:
                        # Loop through subjects by index
                        for subject_index in range(subject_count):
                            subject_name = client.get_subject_name(subject_index)
                            if subject_name:
                                self.logger.debug(f"  Subject: {subject_name}")

                                # Get the root segment
                                root_segment = client.get_subject_root_segment_name(subject_name)

                                # Get the global translation (position)
                                try:
                                    translation = client.get_segment_global_translation(subject_name, root_segment)

                                    # CORRECTED LOGIC:
                                    # translation[0] is the numpy array (X, Y, Z)
                                    # translation[1] is the occlusion flag (True if not occluded, False if occluded)
                                    # We only need to check the occlusion flag.
                                    if translation is not None:  # This means data exists and is not occluded
                                        pos_x, pos_y, pos_z = translation
                                        self.callback(pos_x, pos_y, pos_z)
                                        self.logger.debug(
                                            f"    Position (mm): X={pos_x:.2f}, Y={pos_y:.2f}, Z={pos_z:.2f}")
                                    else:
                                        self.logger.warning(f"    Position (mm): Occluded or no data")

                                except Exception as e:
                                    self.logger.error(f"    Could not get segment data: {e}")
                                    break

                else:
                    time.sleep(0.01)

        except KeyboardInterrupt:
            self.logger.error("\nScript interrupted by user.")
        except Exception as e:
            self.logger.error(f"An unexpected error occurred: {e}")

        finally:
            if client.is_connected():
                client.disconnect()
                self.logger.info("Disconnected from Vicon server.")
