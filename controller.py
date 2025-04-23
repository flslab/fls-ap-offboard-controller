import logging

from pymavlink import mavutil
import time


BLUE = "\033[94m"
RESET = "\033[0m"


def create_logger(name, level=logging.INFO):
    logger = logging.getLogger(name)
    logger.setLevel(level)

    logger.propagate = False

    ch = logging.StreamHandler()
    ch.setLevel(level)

    formatter = logging.Formatter(BLUE+'%(name)s - %(levelname)s - %(message)s'+RESET)
    ch.setFormatter(formatter)

    if not logger.hasHandlers():
        logger.addHandler(ch)

    return logger


class Controller:
    def __init__(self, device="/dev/ttyAMA0", baudrate=57600):
        self.device = device
        self.baudrate = baudrate
        self.master = None
        self.logger = create_logger("Controller")
        self.is_armed = False
        self.connected = False

    def connect(self):
        self.master = mavutil.mavlink_connection(self.device, baud=self.baudrate)

        self.logger.info("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        self.connected = True
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
        mode = 'GUIDED'
        self.logger.info(f"Setting mode to {mode}")
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        ack = False
        while not ack:
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            if ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                self.logger.info(f"{mode} mode set")
                ack = True

    def wait_for_command_ack(self, command, timeout=5):
        """Wait for command acknowledgement"""
        start = time.time()
        while time.time() - start < timeout:
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if msg and msg.command == command:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return True
                else:
                    self.logger.error(f"Command {command} failed with result: {msg.result}")
                    return False

        self.logger.error(f"No ACK received for command {command}")
        return False

    def arm(self):
        """Arm the vehicle"""
        self.logger.info("Arming motors")

        # Send arming command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        # Wait for ACK
        ack = self.wait_for_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if ack:
            self.is_armed = True
            self.logger.info("Vehicle armed")
            return True
        else:
            self.logger.error("Failed to arm")
            return False

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
        ack = self.wait_for_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)


    # Land the drone
    def land(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        self.logger.info("Landing command sent")

    def disarm(self):
        """Disarm the vehicle"""
        if not self.connected:
            self.logger.error("Not connected to vehicle")
            return False

        self.logger.info("Disarming motors")

        # Send disarming command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

        # Wait for ACK
        ack = self.wait_for_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if ack:
            self.is_armed = False
            self.logger.info("Vehicle disarmed")
            return True
        else:
            self.logger.error("Failed to disarm")
            return False


    def set_mode(self, mode):
        """Set the flight mode"""
        self.master.set_mode_apm(mode)
        print(self.master.wait_heartbeat())

        time.sleep(5)
        # custom_mode = self.master.mav.flightmode
        # mode = self.master.flightmode  # human-readable mode string, if available
        # print(f"Current mode: {mode}")
        # if not self.connected:
        #     self.logger.error("Not connected to vehicle")
        #     return False
        #
        # # Map the mode name to mode ID
        # mode_mapping = {
        #     'STABILIZE': 0,
        #     'ACRO': 1,
        #     'ALT_HOLD': 2,
        #     'AUTO': 3,
        #     'GUIDED': 4,
        #     'LOITER': 5,
        #     'RTL': 6,
        #     'CIRCLE': 7,
        #     'POSITION': 8,
        #     'LAND': 9,
        #     'GUIDED_NOGPS': 20,
        # }
        #
        # if mode not in mode_mapping:
        #     self.logger.error(f"Unknown mode: {mode}")
        #     return False
        #
        # mode_id = mode_mapping[mode]
        #
        # # Send mode change command
        # self.master.mav.command_long_send(
        #     self.master.target_system,
        #     self.master.target_component,
        #     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        #     0,
        #     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        #     mode_id,
        #     0, 0, 0, 0, 0
        # )
        #
        # # Wait for ACK
        # ack = self.wait_for_command_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
        # if ack:
        #     self.mode = mode
        #     self.logger.info(f"Mode changed to {mode}")
        #     return True
        # else:
        #     self.logger.error(f"Failed to change mode to {mode}")
        #     return False


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
    c.connect()
    c.request_data()
    c.set_mode('GUIDED')
    # c.set_guided_mode()
    c.arm()
    time.sleep(5)
    # c.takeoff(1.0)
    # time.sleep(3)
    # c.land()
    c.disarm()
