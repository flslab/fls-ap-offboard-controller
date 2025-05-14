import argparse
import logging

from pymavlink import mavutil
import time

from led import MovingDotLED

BLUE = "\033[94m"
RESET = "\033[0m"


def create_logger(name, level=logging.INFO):
    logger = logging.getLogger(name)
    logger.setLevel(level)

    logger.propagate = False

    ch = logging.StreamHandler()
    ch.setLevel(level)

    formatter = logging.Formatter(BLUE + '%(name)s - %(levelname)s - %(message)s' + RESET)
    ch.setFormatter(formatter)

    if not logger.hasHandlers():
        logger.addHandler(ch)

    return logger


class Controller:
    def __init__(self, flight_duration, voltage_threshold, device="/dev/ttyAMA0", baudrate=57600):
        self.device = device
        self.baudrate = baudrate
        self.master = None
        self.logger = create_logger("Controller")
        self.is_armed = False
        self.connected = False
        self.flight_duration = flight_duration
        self.voltage_threshold = voltage_threshold

    def connect(self):
        self.master = mavutil.mavlink_connection(self.device, baud=self.baudrate)

        self.logger.info("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        self.connected = True
        self.logger.info(
            f"Heartbeat from system (system {self.master.target_system} component {self.master.target_component})")

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

    def reboot(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,  # confirmation
            1,  # param1: 1=reboot autopilot
            0, 0, 0, 0, 0, 0  # unused parameters
        )

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

    def arm_with_retry(self):
        """Arm the vehicle with retry"""
        if not self.connected:
            self.logger.error("Not connected to vehicle")
            return False

        self.logger.info("Arming motors")

        # Try up to 3 times to arm
        for attempt in range(3):
            # Override any pre-arm failsafe checks
            # 21196 as the 6th param is a magic number that forces arming
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )

            # Wait for armed status
            start = time.time()
            while time.time() - start < 5:
                heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if heartbeat and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    self.is_armed = True
                    self.logger.info("Vehicle armed")
                    return True

            self.logger.warning(f"Arm attempt {attempt + 1} failed, retrying...")

        self.logger.error("Failed to arm after multiple attempts")
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
        # self.master.set_mode_apm(mode)
        if not self.connected:
            self.logger.error("Not connected to vehicle")
            return False

        # Map the mode name to mode ID
        mode_mapping = {
            'STABILIZE': 0,
            'ACRO': 1,
            'ALT_HOLD': 2,
            'AUTO': 3,
            'GUIDED': 4,
            'LOITER': 5,
            'RTL': 6,
            'CIRCLE': 7,
            'POSITION': 8,
            'LAND': 9,
            'GUIDED_NOGPS': 20,
        }

        if mode not in mode_mapping:
            self.logger.error(f"Unknown mode: {mode}")
            return False

        mode_id = mode_mapping[mode]

        # Send mode change command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0, 0, 0, 0, 0
        )

        print(self.master.wait_heartbeat())

    def send_motor_test(self, motor_index, throttle_type, throttle_value, duration):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0,  # confirmation
            motor_index,  # param1: motor index
            throttle_type,  # param2: throttle type
            throttle_value,  # param3: throttle value
            duration,  # param4: timeout (seconds)
            0, 0, 0  # param5-7: unused
        )

    def test_motors(self):
        self.logger.info("Testing motors")
        for i in range(4):
            self.send_motor_test(i+1, throttle_type=0, throttle_value=10, duration=1)
            time.sleep(0.25)

    def watch_battery(self):
        start = time.perf_counter()

        while True:
            elapsed = time.perf_counter() - start

            if 0 < self.flight_duration <= elapsed:
                break

            msg = self.master.recv_match(type=['BATTERY_STATUS'], blocking=True, timeout=1)
            voltage = current = 'N/A'

            if msg:
                if msg.get_type() == 'BATTERY_STATUS':
                    voltage_raw = msg.voltages[0]
                    voltage = voltage_raw / 1000.0 if voltage_raw != 65535 else 'N/A'
                    current = msg.current_battery / 100.0 if msg.current_battery != -1 else 'N/A'

                    if isinstance(voltage, float) and voltage < self.voltage_threshold:
                        self.logger.info(f"Attempt to land due to low battery.")
                        break

            self.logger.info(f"{elapsed:.2f}s | V: {voltage} V | I: {current} A")


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--test-motors", action="store_true")
    arg_parser.add_argument("--reboot", action="store_true")
    arg_parser.add_argument("--led", action="store_true")
    arg_parser.add_argument("--land", action="store_true")
    arg_parser.add_argument("-t", "--duration", type=float, default=15.0)
    arg_parser.add_argument("--voltage", type=float, default=7.35)
    args = arg_parser.parse_args()

    c = Controller(flight_duration=args.duration, voltage_threshold=args.voltage)
    c.connect()

    if args.reboot:
        c.reboot()
        exit()

    if args.land:
        c.land()
        time.sleep(10)
        c.disarm()
        exit()

    c.request_data()

    if args.test_motors:
        c.test_motors()
        time.sleep(5)

    c.set_mode('GUIDED')

    c.arm_with_retry()
    if args.led:
        led = MovingDotLED()
        led.start()

    time.sleep(5)

    c.takeoff(1.0)

    c.watch_battery()

    c.land()

    time.sleep(10)
    c.disarm()

    if args.led:
        led.stop()
