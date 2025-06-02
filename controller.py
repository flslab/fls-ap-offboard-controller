import argparse
import logging
import struct
import subprocess
import time
import math
import mmap
from threading import Thread

import numpy as np
from pymavlink import mavutil

from log import LoggerFactory

linear_accel_cov = 0.01
angular_vel_cov = 0.01


def truncate(f, n):
    return int(f * 10**n) / 10**n


class Controller:
    def __init__(self, flight_duration, voltage_threshold, takeoff_altitude, log_level=logging.INFO, sim=False,
                 device="/dev/ttyAMA0",
                 baudrate=57600):
        self.device = device
        self.baudrate = baudrate
        self.master = None
        self.logger = LoggerFactory("Controller", level=log_level).get_logger()
        self.is_armed = False
        self.connected = False
        self.flight_duration = flight_duration
        self.voltage_threshold = voltage_threshold
        self.takeoff_altitude = takeoff_altitude
        self.start_time = time.time()
        self.battery_low = False
        self.running_position_estimation = False
        self.running_battery_watcher = False
        self.sim = sim

    def connect(self):
        if self.sim:
            self.master = mavutil.mavlink_connection("udpin:127.0.0.1:14551")
        else:
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
        time.sleep(5)

    def reboot(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,  # confirmation
            1,  # param1: 1=reboot autopilot
            0, 0, 0, 0, 0, 0  # unused parameters
        )

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
            while time.time() - start < 2:
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
    def takeoff(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            self.takeoff_altitude
        )
        self.logger.info(f"Takeoff command sent to {self.takeoff_altitude} meters")
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

        while True:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)

            if msg is None:
                continue

            if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == 0:
                self.logger.info("Vehicle is disarmed")
                break

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

        ack = self.wait_for_command_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
        heartbeat = self.master.wait_heartbeat()
        if ack and heartbeat.custom_mode == mode_id:
            self.logger.info(f"Mode changed to {mode}")
            return True
        else:
            self.logger.error(f"Failed to change mode to {mode}")
            return False

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
            self.send_motor_test(i + 1, throttle_type=0, throttle_value=10, duration=1)
            time.sleep(0.25)

    def watch_battery(self):
        start = time.perf_counter()

        while self.running_battery_watcher:
            elapsed = time.perf_counter() - start

            msg = self.master.recv_match(type=['BATTERY_STATUS'], blocking=True, timeout=1)
            voltage = current = 'N/A'

            if msg:
                if msg.get_type() == 'BATTERY_STATUS':
                    voltage_raw = msg.voltages[0]
                    voltage = voltage_raw / 1000.0 if voltage_raw != 65535 else 'N/A'
                    current = msg.current_battery / 100.0 if msg.current_battery != -1 else 'N/A'

                    if isinstance(voltage, float) and voltage < self.voltage_threshold:
                        self.battery_low = True
                        self.logger.warning(f"Failsafe triggered due to low battery")
                        break

            self.logger.info(f"{elapsed:.2f}s | V: {voltage} V | I: {current} A")

    def send_trajectory_message(self, point_num, pos, vel, acc, time_horizon):
        """Send a trajectory setpoint using MAVLink TRAJECTORY message."""
        # MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS = 0
        type_mask = 0  # Use position, velocity and acceleration

        # Create the message - using local coordinates since there's no GPS
        self.master.mav.trajectory_representation_waypoints_send(
            time_us=int(time.time() * 1000000),  # Current time in microseconds
            valid_points=1,  # Sending one point at a time
            pos_x=[pos[0], 0, 0, 0, 0],  # X position in meters (NED frame)
            pos_y=[pos[1], 0, 0, 0, 0],  # Y position in meters
            pos_z=[pos[2], 0, 0, 0, 0],  # Z position in meters
            vel_x=[vel[0], 0, 0, 0, 0],  # X velocity in m/s
            vel_y=[vel[1], 0, 0, 0, 0],  # Y velocity in m/s
            vel_z=[vel[2], 0, 0, 0, 0],  # Z velocity in m/s
            acc_x=[acc[0], 0, 0, 0, 0],  # X acceleration in m/s^2
            acc_y=[acc[1], 0, 0, 0, 0],  # Y acceleration in m/s^2
            acc_z=[acc[2], 0, 0, 0, 0],  # Z acceleration in m/s^2
            pos_yaw=[0, 0, 0, 0, 0],  # Yaw angle in rad
            vel_yaw=[0, 0, 0, 0, 0],  # Yaw velocity in rad/s
            command=[mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0],  # Command for each point
            time_horizon=[time_horizon, 0, 0, 0, 0]  # Time horizon for this point in seconds
        )
        self.logger.debug(f"Sent trajectory point {point_num}: Pos={pos}, Vel={vel}, Acc={acc}")

    def send_position_target(self, x, y, z):
        """
        Sends waypoints in local NED frame
        X is forward, Y is right, Z is down with origin fixed relative to ground

        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        """
        self.logger.debug(f"Sending position {x} {y} {z}")
        self.master.mav.set_position_target_local_ned_send(
            int((time.time() - self.start_time) * 1000),  # milliseconds since start
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b100111111000,  # only x, y, z position and yaw
            x, y, z,
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0  # yaw, yaw_rate
        )

    def send_velocity_target(self, vx, vy, vz):
        """
        Sends waypoints in local NED frame
        X is forward, Y is right, Z is down with origin fixed relative to ground

        MAV_FRAME_BODY_OFFSET_NED
        """
        self.logger.debug(f"Sending velocity {vx} {vy} {vz}")
        self.master.mav.set_position_target_local_ned_send(
            int((time.time() - self.start_time) * 1000),  # milliseconds since start
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b000111000111,  # only velocity and yaw
            0, 0, 0,
            vx, vy, vz,  # velocity
            0, 0, 0,  # acceleration
            0, 0  # yaw, yaw_rate
        )

    def send_acceleration_target(self, ax, ay, az):
        """
        Sends waypoints in local NED frame
        X is forward, Y is right, Z is down with origin fixed relative to ground
        """
        self.logger.debug(f"Sending acceleration {ax} {ay} {az}")
        self.master.mav.set_position_target_local_ned_send(
            int((time.time() - self.start_time) * 1000),  # milliseconds since start
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b110000111111,  # only acceleration and yaw
            0, 0, 0,
            0, 0, 0,  # velocity
            ax, ay, az,  # acceleration
            0, 0  # yaw, yaw_rate
        )

    def send_trajectory_from_file(self, file_path):
        """Read and send a trajectory."""
        import pandas as pd
        df = pd.read_csv(file_path)

        # Extract coordinates and vectors
        x = df['x'].values
        y = df['y'].values
        z = df['z'].values

        t = df['time'].values

        # vx = df['velocity_x'].values
        # vy = df['velocity_y'].values
        # vz = df['velocity_z'].values
        # speed = df['speed'].values
        #
        # ax = df['acceleration_x'].values
        # ay = df['acceleration_y'].values
        # az = df['acceleration_z'].values
        # am = df['acceleration_magnitude'].values

        # Calculate time interval between points
        time_interval = t[1] - t[0]
        point_count = len(t)

        # Send each point in the trajectory
        for j in range(2):
            for i in range(point_count):
                if self.battery_low:
                    return
                self.send_position_target(x[i] * 3 - x[0] * 3, z[i] * 3 - z[0] * 3, -1 - y[i] * 3)
                time.sleep(1 / 10)

    def test_trajectory(self, x=0, y=0, z=0):
        self.logger.info("Sending")
        points = [(x, y, -self.takeoff_altitude - z)]

        for j in range(1):
            for point in points:
                for i in range(int(self.flight_duration * 10)):
                    if self.battery_low:
                        return
                    self.send_position_target(point[0], point[1], -1)
                    time.sleep(1 / 10)

    def test_s_trajectory(self):
        self.logger.info("Sending")
        # points = [(0.6, 1.7, 0), (0.35, 2, 0), (0, 1.7, 0), (0.15, 1.2, 0), (0.35, 1, 0),
        #  (0.55, .8, 0), (0.7, 0.3, 0), (0.35, 0, 0), (0.1, 0.3, 0)]
        points = [(0, 0, 0), (3, 0, 0)]

        for j in range(1):
            for point in points:
                for i in range(200):
                    if self.battery_low:
                        return
                    self.send_position_target(point[0], point[1], -1 - point[2])
                    time.sleep(1 / 20)

    def circular_trajectory(self):
        radius = 0.5  # 1m diameter
        center_x = 0
        center_y = 0
        altitude = -1.0  # NED frame: -1 means 1 meter above ground
        frequency = 20  # Hz
        period = 3  # seconds per revolution
        angular_velocity = 2 * math.pi / period

        start_time = time.time()

        while not self.battery_low:
            t = time.time() - start_time
            if 0 < self.flight_duration <= t:
                break

            x = center_x + radius * math.cos(angular_velocity * t)
            y = center_y + radius * math.sin(angular_velocity * t)
            z = altitude
            self.send_position_target(x, y, z)
            time.sleep(1 / frequency)

    def send_position_estimation(self):
        position_size = 1 + 3 + 6 * 4  # 1 byte + 3 byte padding + 6 floats (4 bytes each)
        shm_name = "/pos_shared_mem"
        shm_fd = open(f"/dev/shm{shm_name}", "r+b")  # Open shared memory
        shm_map = mmap.mmap(shm_fd.fileno(), position_size, access=mmap.ACCESS_READ)
        reset_counter = 1

        while self.running_position_estimation:
            data = shm_map[:position_size]  # Read 28 bytes
            valid = struct.unpack("<?", data[:1])[0]  # Extract the validity flag (1 byte)

            if valid:
                x, y, z, roll, pitch, yaw = struct.unpack("<6f", data[4:28])
                # x, y, z = struct.unpack("<3f", data[4:16])
                x = truncate(x, 3)
                y = truncate(y, 3)
                z = truncate(z, 3)
                # self.logger.debug(f"Sending position estimation: ({-y}, {x}, {-z} | {roll}, {pitch}, {yaw})")
                self.logger.debug(f"Sending position estimation: ({y}, {-x}, {-z})")

                rpy_rad = np.array([roll, pitch, yaw])

                # Setup covariance data, which is the upper right triangle of the covariance matrix, see here: https://files.gitter.im/ArduPilot/VisionProjects/1DpU/image.png
                # Attemp #01: following this formula https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/src/base_realsense_node.cpp#L1406-L1411
                cov_pose = linear_accel_cov * pow(10, 3 - int(3))
                cov_twist = angular_vel_cov * pow(10, 1 - int(3))
                # covariance = np.array([cov_pose, 0, 0, 0, 0, 0,
                #                        cov_pose, 0, 0, 0, 0,
                #                        cov_pose, 0, 0, 0,
                #                        cov_twist, 0, 0,
                #                        cov_twist, 0,
                #                        cov_twist])

                covariance = np.array([0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0,
                                       0, 0, 0, 0,
                                       0, 0, 0,
                                       0, 0,
                                       0])

                self.master.mav.vision_position_estimate_send(
                    int((time.time()) * 1000000),
                    y,  # X y
                    -x,  # Y -x
                    0,  # Z -z (down is negative)
                    0,  #rpy_rad[0],  # Roll angle
                    0,  #rpy_rad[1],  # Pitch angle
                    0,  #rpy_rad[2],  # Yaw angle
                    covariance,  # Row-major representation of pose 6x6 cross-covariance matrix
                    reset_counter  # Estimate reset counter. Increment every time pose estimate jumps.
                )
                # self.master.mav.vision_position_estimate_send(
                #     int((time.time()) * 1000000),
                #     0,  # X y
                #     0,  # Y -x
                #     0,  # Z (down is negative)
                #     0,  # Roll angle
                #     0,  # Pitch angle
                #     0,  # Yaw angle
                #     covariance,  # Row-major representation of pose 6x6 cross-covariance matrix
                #     reset_counter  # Estimate reset counter. Increment every time pose estimate jumps.
                # )
                q = [1.0, 0.0, 0.0, 0.0]  # identity quaternion (w, x, y, z)
                pose_cov = [0.0] * 21
                vel_cov = [0.0] * 21
                # self.master.mav.odometry_send(
                #     time_usec=usec,
                #     frame_id=1,          # MAV_FRAME_LOCAL_NED
                #     child_frame_id=1,    # arbitrary, just be consistent
                #     x=x, y=y, z=z,
                #     q=q,
                #     vx=0, vy=0, vz=0,
                #     rollspeed=0,
                #     pitchspeed=0,
                #     yawspeed=0,
                #     pose_covariance=pose_cov,
                #     velocity_covariance=vel_cov,
                #     reset_counter=0,
                #     estimator_type=0,   # MAV_ESTIMATOR_TYPE_NAIVE
                #     quality=0           # 0-100, optional
                # )
            else:
                pass
                # print("Invalid data received")

            time.sleep(1 / args.fps)

    def start_flight(self):
        battery_thread = Thread(target=self.watch_battery, daemon=True)

        time.sleep(3)
        c.takeoff()
        time.sleep(5)

        # flight_thread = Thread(target=self.send_trajectory_from_file, args=(args.trajectory,))
        flight_thread = Thread(target=self.test_trajectory, args=(0, 0, 0))
        # flight_thread = Thread(target=self.circular_trajectory)

        self.running_battery_watcher = True
        battery_thread.start()
        flight_thread.start()

        flight_thread.join()
        self.running_battery_watcher = False
        battery_thread.join()

    def stop(self):
        self.land()

        if args.led:
            led.stop()


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--test-motors", action="store_true", help="test motors and exit")
    arg_parser.add_argument("--reboot", action="store_true", help="reboot")
    arg_parser.add_argument("--led", action="store_true", help="turn on the leds")
    arg_parser.add_argument("--land", action="store_true", help="land and exit")
    arg_parser.add_argument("--status", action="store_true", help="show battery voltage and current")
    arg_parser.add_argument("--debug", action="store_true", help="show debug logs")
    arg_parser.add_argument("--sim", action="store_true", help="connect to simulator")
    arg_parser.add_argument("--localize", action="store_true", help="localize using camera")
    arg_parser.add_argument("-t", "--duration", type=float, default=15.0, help="flight duration in seconds")
    arg_parser.add_argument("--fps", type=int, default=120, help="position estimation rate")
    arg_parser.add_argument("--takeoff-altitude", type=float, default=1.0, help="takeoff altitude in meter")
    arg_parser.add_argument("--voltage", type=float, default=7.4,
                            help="critical battery voltage threshold to land when reached")
    arg_parser.add_argument("--trajectory", type=str, help="path to trajectory file to follow")
    arg_parser.add_argument("--simple-takeoff", action="store_true", help="takeoff and land")
    args = arg_parser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    c = Controller(
        takeoff_altitude=args.takeoff_altitude,
        sim=args.sim,
        log_level=log_level,
        flight_duration=args.duration,
        voltage_threshold=args.voltage,
    )
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

    if args.status:
        c.watch_battery()
        exit()

    if args.test_motors:
        c.test_motors()
        exit()

    localize_thread = Thread(target=c.send_position_estimation)

    if args.localize:
        lat = 12345
        lon = 12345
        alt = 0
        c.master.mav.set_gps_global_origin_send(1, lat, lon, alt)
        c.master.mav.set_home_position_send(1, lat, lon, alt, 0, 0, 0, [1, 0, 0, 0], 0, 0, 1)
        c.running_position_estimation = True
        c_process = subprocess.Popen([
            "/home/fls/fls-marker-localization/build/eye",
            "-t", str(30 + args.duration),
            "--config", "/home/fls/fls-marker-localization/build/camera_config.json",
            "-s",
            "--save-rate", "10",
            "--brightness", "0.5",
            "--contrast", "2.5",
            "--exposure", "500",
            "--fps", str(args.fps),
        ])

        time.sleep(2)
        localize_thread.start()

    if not c.set_mode('GUIDED'):
        pass
        # exit()

    if not c.arm_with_retry():
        pass
        # exit()

    if args.led:
        from led import MovingDotLED

        led = MovingDotLED()
        led.start()

    # time.sleep(10)
    c.start_flight()
    c.stop()

    if args.localize:
        c.running_position_estimation = False
        localize_thread.join()
