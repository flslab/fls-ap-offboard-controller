import logging

from pymavlink import mavutil
import time


class Controller:
    def __init__(self, device="/dev/ttyAMA0", baudrate=57600):
        self.device = device
        self.baudrate = baudrate
        self.master = None
        self.logger = logging.getLogger('Controller')
        self.logger.setLevel(logging.INFO)

    def connect(self):
        self.master = mavutil.mavlink_connection(self.device, baud=self.baudrate)

        self.logger.info("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        self.logger.info(f"Heartbeat from system (system {self.master.target_system} component {self.master.target_component})")

    def request_data(self):
        self.logger.info("Requesting parameters...")
        self.master.mav.param_request_list_send(
            self.master.target_system, self.master.target_component
        )

        time.sleep(2)

        self.logger.info("Setting up data streams...")
        for i in range(0, 3):  # Try a few times to make sure it gets through
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4,  # 4 Hz
                1  # Start sending
            )
            time.sleep(0.5)

        self.logger.info("Waiting for system initialization...")
        time.sleep(3)

    def set_guided_mode(self):
        mode = 'GUIDED_NOGPS'
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        ack = False
        while not ack:
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            if ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                self.logger.info("GUIDED mode set")
                ack = True

    # Arm the drone
    def arm(self):
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        self.logger.info("Drone armed")

    # Take off to target altitude (in meters)
    def takeoff(self, target_altitude):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            target_altitude
        )
        self.logger.info(f"Takeoff command sent to {target_altitude} meters")

    # Land the drone
    def land(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        self.logger.info("Landing command sent")

    # Main sequence


    # # Wait until near 1 meter (or timeout)
    # print("Waiting to reach target altitude...")
    # start_time = time.time()
    # while True:
    #     msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    #     if msg:
    #         alt = msg.relative_alt / 1000.0  # mm to meters
    #         print(f"Altitude: {alt:.2f}m")
    #         if alt > 0.95:
    #             print("Target altitude reached")
    #             break
    #     if time.time() - start_time > 20:
    #         print("Timeout while waiting for altitude")
    #         break
    #
    # # Wait a bit before landing
    # time.sleep(3)
    # land()
    # print("Mission complete.")

if __name__ == "__main__":
    c = Controller()
    c.set_guided_mode()
    c.arm()
    c.takeoff(1.0)
    time.sleep(3)
    c.land()