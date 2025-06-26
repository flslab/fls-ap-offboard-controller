import argparse
import json
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
    return int(f * 10 ** n) / 10 ** n


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (in radians) to quaternion (x, y, z, w).
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, y, z]


# Class for formatting the Mission Item.
class MissionItem:
    def __init__(self, i, current, x, y, z):
        self.seq = i
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.auto = 1
        self.param1 = 0.0
        self.param2 = 0.1
        self.param3 = 0.0
        self.param4 = math.nan
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0  # The MAV_MISSION_TYPE value for MAV_MISSION_TYPE_MISSION


class Controller:
    def __init__(self, flight_duration, voltage_threshold, takeoff_altitude, land_altitude, log_level=logging.INFO,
                 sim=False,
                 device="/dev/ttyAMA0",
                 baudrate=115200):
        self.device = device
        self.baudrate = baudrate
        self.master = None
        self.logger = LoggerFactory("Controller", level=log_level).get_logger()
        self.is_armed = False
        self.connected = False
        self.flight_duration = flight_duration
        self.voltage_threshold = voltage_threshold
        self.takeoff_altitude = takeoff_altitude
        self.land_altitude = land_altitude
        self.start_time = time.time()
        self.battery_low = False
        self.running_position_estimation = False
        self.running_battery_watcher = False
        self.sim = sim
        self.initial_yaw = 0
        self.mission_items = []

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

        msg = self.master.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            yaw_rad = msg.yaw
            self.logger.info(f"initial yaw in radians: {yaw_rad}")
            self.initial_yaw = yaw_rad

    def reboot(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,  # confirmation
            1,  # param1: 1=reboot autopilot
            0, 0, 0, 0, 0, 0  # unused parameters
        )

    def wait_for_command_ack(self, ack_type='COMMAND_ACK', command=None, timeout=5):
        """Wait for command acknowledgement"""
        start = time.time()
        while time.time() - start < timeout:
            msg = self.master.recv_match(type=ack_type, blocking=True, timeout=1)
            if command is None:
                return True
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
        ack = self.wait_for_command_ack(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
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
        ack = self.wait_for_command_ack(command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

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

    def custom_land(self):
        delta_z = self.takeoff_altitude - self.land_altitude
        for i in range(50):
            self.send_position_target(0, 0, -self.takeoff_altitude + (i / 50) * delta_z)
            time.sleep(1 / 10)

        for i in range(20):
            self.send_position_target(0, 0, -self.land_altitude)
            time.sleep(1 / 10)
        self.land()

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
        ack = self.wait_for_command_ack(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
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

        ack = self.wait_for_command_ack(command=mavutil.mavlink.MAV_CMD_DO_SET_MODE)
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
                        self.logger.warning(f"Failsafe triggered due to low battery ({voltage} V)")
                        break

            self.logger.debug(f"{elapsed:.2f}s | V: {voltage} V | I: {current} A")

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
            self.initial_yaw, 0  # yaw, yaw_rate
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
            0b100111000111,  # only velocity and yaw
            0, 0, 0,
            vx, vy, vz,  # velocity
            0, 0, 0,  # acceleration
            self.initial_yaw, 0  # yaw, yaw_rate
        )

    def send_position_velocity_target(self, x, y, z, vx, vy, vz):
        """
        Sends waypoints in local NED frame
        X is forward, Y is right, Z is down with origin fixed relative to ground

        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        """
        self.logger.debug(f"Sending position velocity {x} {y} {z} | {vx} {vy} {vz}")
        self.master.mav.set_position_target_local_ned_send(
            int((time.time() - self.start_time) * 1000),  # milliseconds since start
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b100111000000,  # x, y, z position, velocity and yaw
            x, y, z,
            vx, vy, vz,  # velocity
            0, 0, 0,  # acceleration
            self.initial_yaw, 0  # yaw, yaw_rate
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
            self.initial_yaw, 0  # yaw, yaw_rate
        )

    def send_attitude_target_deg(self, roll_deg, pitch_deg, yaw_deg, thrust=0.5):
        """
        Sends a SET_ATTITUDE_TARGET message to ArduPilot using degrees for angles.

        Args:
            master: pymavlink MAVLink connection (from mavutil.mavlink_connection)
            roll_deg: desired roll in degrees
            pitch_deg: desired pitch in degrees
            yaw_deg: desired yaw in degrees
            thrust: throttle value between 0.0 and 1.0
        """
        # Convert degrees to quaternion
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        q = euler_to_quaternion(roll, pitch, yaw)

        # print(q)
        # Send SET_ATTITUDE_TARGET
        self.master.mav.set_attitude_target_send(
            int((time.time() - self.start_time) * 1000),
            self.master.target_system,
            self.master.target_component,
            0b00000111,
            q,  # [w, x, y, z]
            0, 0, 0,  # body roll/pitch/yaw rates (ignored)
            thrust  # Thrust (0-1)
        )

    def upload_mission(self, mission_items):
        n = len(mission_items)

        # Send the number of mission items
        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            n,
            0
        )

        self.wait_for_command_ack(ack_type="MISSION_REQUEST")

        for waypoint in mission_items:

            # Send each mission item
            self.master.mav.mission_item_send(
                self.master.target_system,  # Target System
                self.master.target_component,  # Target Component
                waypoint.seq,  # Sequence
                waypoint.frame,  # Frame
                waypoint.command,  # Command
                waypoint.current,  # Current
                waypoint.auto,  # Auto continue
                waypoint.param1,  # Hold Time
                waypoint.param2,  # Accept Radius
                waypoint.param3,  # Pass Radius
                waypoint.param4,  # Yaw
                waypoint.param5,  # Local X
                waypoint.param6,  # Local Y
                waypoint.param7,  # Local Z
                waypoint.mission_type  # Mission Type
            )

            if waypoint != mission_items[n - 1]:
                self.wait_for_command_ack(ack_type="MISSION_REQUEST")

        self.wait_for_command_ack(ack_type="MISSION_ACK")

    def send_trajectory_from_file(self, file_path):
        with open(file_path, "r") as f:
            trajectory = json.load(f)

        fps = trajectory["fps"]
        start_position = trajectory["start_position"]
        segments = trajectory["segments"]

        #  go to start position
        for _ in range(10):
            x, y, z = start_position
            self.send_position_target(x, y, -self.takeoff_altitude - z)
            time.sleep(1/10)

        for i in range(3):
            for segment in segments:
                positions = segment["position"]
                velocities = segment["velocity"]
                state = segment["state"]
                if state == "LIT":
                    led.turn_on()
                else:
                    led.clear()

                for p, v in zip(positions, velocities):
                    if self.battery_low:
                        return
                    x, y, z = p
                    vx, vy, vz = v
                    self.send_position_velocity_target(x, y, -self.takeoff_altitude-z, vx, vy, -vz)
                    time.sleep(1/fps)

        led.clear()

        #  go to start position
        for _ in range(10):
            x, y, z = start_position
            self.send_position_target(x, y, -self.takeoff_altitude - z)
            time.sleep(1 / 10)


    def send_trajectory_from_file_(self, file_path):
        """Read and send a trajectory."""
        import pandas as pd
        df = pd.read_csv(file_path)

        # Extract coordinates and vectors
        x = df['x'].values
        y = df['y'].values
        z = df['z'].values

        t = df['time'].values

        vx = df['velocity_x'].values
        vy = df['velocity_y'].values
        vz = df['velocity_z'].values
        # speed = df['speed'].values
        #
        # ax = df['acceleration_x'].values
        # ay = df['acceleration_y'].values
        # az = df['acceleration_z'].values
        # am = df['acceleration_magnitude'].values

        # Calculate time interval between points
        dt = t[1] - t[0]
        point_count = len(t)
        repeat_point = 1

        y_scale = 1
        z_scale = 1

        min_x = min(x)
        max_x = max(x)
        range_x = max_x - min_x

        # Send each point in the trajectory
        for j in range(3):
            for i in range(point_count):
                if self.battery_low:
                    return

                _x = 0
                _y = (x[i] - range_x / 2) * y_scale
                _z = - self.takeoff_altitude - (z[i] - z[0]) * z_scale

                _vx = 0
                _vy = vx[i] * y_scale
                _vz = vz[i] * z_scale

                if i == 0:
                    if j == 0 or 'loop' not in args.trajectory:
                        self.logger.info(f"Go to start coordinates: {_x}, {_y}, {_z}")
                        for _ in range(20):
                            # wait at the start
                            self.send_position_velocity_target(_x, _y, _z, 0, 0, 0)
                            time.sleep(dt)

                    led.turn_on()
                    self.logger.info(f"Start to follow the path")

                # self.send_position_target(_x, _y, _z)
                self.send_position_velocity_target(_x, _y, _z, _vx, _vy, _vz)
                # self.send_velocity_target(_vx, _vy, _vz)

                time.sleep(dt)

                # if 'loop' in args.trajectory and i == 239:
                #     led.clear()

            led.clear()
            if 'loop' not in args.trajectory:
                for _ in range(20):
                    # wait at the end
                    self.send_position_velocity_target(_x, _y, _z, 0, 0, 0)
                    time.sleep(dt)

        self.logger.info(f"Path completed")
        self.logger.info("Prepare to land")
        for _ in range(10):
            self.send_position_target(0, 0, -self.takeoff_altitude)
            time.sleep(1 / 10)

    def fly_figure_eight(self, rounds=3, radius=.2, speed=.2):
        for _ in range(20):
            self.send_position_target(0, 0, -self.takeoff_altitude)
            time.sleep(0.1)

        omega = speed / radius

        for n in range(rounds):
            start_time = time.time()
            while True:
                elapsed_time = time.time() - start_time
                t = omega * elapsed_time

                if t > 4 * math.pi:
                    break

                # Parametric equations for a figure-8
                x = radius * math.sin(t)
                y = radius * math.sin(t) * math.cos(t)

                # Derivatives for velocity
                vx = speed * math.cos(t)
                vy = speed * (math.cos(t) ** 2 - math.sin(t) ** 2)
                vz = 0

                self.send_position_velocity_target(0, x, -self.takeoff_altitude - y, 0, vx, -vy)
                time.sleep(0.1)

    def send_mission_from_file(self, file_path):
        """Read and upload a waypoint mission."""

        import pandas as pd
        df = pd.read_csv(file_path)

        # Extract coordinates and vectors
        x = df['x'].values
        y = df['y'].values
        z = df['z'].values

        t = df['time'].values

        vx = df['velocity_x'].values
        vy = df['velocity_y'].values
        vz = df['velocity_z'].values
        # speed = df['speed'].values
        #
        # ax = df['acceleration_x'].values
        # ay = df['acceleration_y'].values
        # az = df['acceleration_z'].values
        # am = df['acceleration_magnitude'].values

        # Calculate time interval between points
        time_interval = t[1] - t[0]
        point_count = len(t)

        __x = 0
        __y = 0
        __z = 0
        dt = 1 / 10

        self.mission_items = []

        self.mission_items.append(MissionItem(0, 0, 0, 0, 1))
        self.mission_items.append(MissionItem(1, 0, -35.363026, 149.165152, 1))
        self.mission_items.append(MissionItem(2, 0, 0, 0, 1))

        # for i in range(point_count):
        #     if args.sim:
        #         _x = (z[i] - z[0]) * 2.5
        #         _y = (x[i] - x[0]) * 2.5
        #         _z = -self.takeoff_altitude
        #     else:
        #         _x = 0
        #         _y = (x[i]) * 2.5 - x[0]
        #         _z = - self.takeoff_altitude - (z[i] - z[0]) * 2.5
        #
        #     # _vx = 0
        #     # _vy = vx[i] * 10
        #     # _vz = vz[i] * 10
        #     # _vx = (_x - __x) / dt
        #     # _vy = (_y - __y) / dt
        #     # _vz = (_z - __z) / dt
        #
        #     # __x = _x
        #     # __y = _y
        #     # __z = _z
        #     # print(_x, _y, _z)
        #     self.mission_items.append(MissionItem(i, 0, _x, _y, _z))
        #     # self.send_position_velocity_target(_z, _y, -1, _vz, _vy, _vx)
        #     # self.send_velocity_target(_vx, _vy, _vz)
        #     time.sleep(dt)

        self.upload_mission(self.mission_items)

    def start_mission(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,  # confirmation
            0, 0, 0, 0, 0, 0, 0  # unused parameters
        )
        self.wait_for_command_ack(command=mavutil.mavlink.MAV_CMD_MISSION_START)

        for mission_item in self.mission_items:
            msg = self.master.recv_match(
                type='MISSION_ITEM_REACHED',
                condition=f'MISSION_ITEM_REACHED.seq == {mission_item.seq}',
                blocking=True
            )
            self.logger.info(msg)

    def generate_pos_vel_path(self, waypoints, target_speed, dt):
        path = []
        for i in range(len(waypoints) - 1):
            start = np.array(waypoints[i])
            end = np.array(waypoints[i + 1])
            direction = end - start
            distance = np.linalg.norm(direction)
            if distance == 0:
                continue
            direction /= distance
            velocity = direction * target_speed
            steps = int(distance / (target_speed * dt))
            for s in range(steps):
                pos = start + direction * s * target_speed * dt
                path.append((pos.tolist(), velocity.tolist()))
        return path

    def test_trajectory(self, x=0, y=0, z=0):
        points = [(x, y, -self.takeoff_altitude - z)]

        for j in range(1):
            for point in points:
                for i in range(int(self.flight_duration * 10)):
                    if self.battery_low:
                        return
                    self.send_position_target(point[0], point[1], point[2])
                    time.sleep(1 / 10)

    def test_trajectory_2(self):
        waypoints = [
            [0, -.1, -self.takeoff_altitude],
            [0, -.1, -self.takeoff_altitude + .6],
            [0, .1, -self.takeoff_altitude + .6]
        ]
        # trajectory = self.generate_pos_vel_path(waypoints, target_speed=1.0, dt=0.05)

        for _ in range(3):
            for p in waypoints:
                for i in range(50):
                    self.send_position_target(*p)
                    time.sleep(1 / 10)

    def test_trajectory_3(self):
        waypoints = [
            [0, 0, -self.takeoff_altitude],
            [0, .2, -self.takeoff_altitude],
            [0, 0, -self.takeoff_altitude]
        ]

        for pos in waypoints:
            for i in range(50):
                self.send_position_target(*pos)
                time.sleep(1 / 10)

    def test_trajectory_4(self):
        waypoints = [
                        [-10, 0, 0],
                        [10, 0, 0],
                        [10, 0, 0],
                        [-10, 0, 0],
                    ] * 5

        waypoints_2 = [
                          [0, 10, 0],
                          [0, -10, 0],
                          [0, -10, 0],
                          [0, 10, 0],
                      ] * 5

        for ori in waypoints:
            for i in range(5):
                self.send_attitude_target_deg(*ori)
                time.sleep(1 / 20)

        for i in range(30):
            self.send_velocity_target(0, 0, 0)
            time.sleep(1 / 20)

        for ori in waypoints_2:
            for i in range(5):
                self.send_attitude_target_deg(*ori)
                time.sleep(1 / 20)
        # angle = 10
        # for j in range(5):
        #     for i in range(angle + 1):
        #         self.send_attitude_target_deg(i, 0, 0)
        #         time.sleep(1 / 20)
        #
        #     for i in range(angle + 1):
        #         self.send_attitude_target_deg(angle - i, 0, 0)
        #         time.sleep(1 / 20)
        #
        #     for i in range(angle + 1):
        #         self.send_attitude_target_deg(-i, 0, 0)
        #         time.sleep(1 / 20)
        #
        #     for i in range(angle + 1):
        #         self.send_attitude_target_deg(-angle + i, 0, 0)
        #         time.sleep(1 / 20)
        #
        # for i in range(20):
        #     self.send_attitude_target_deg(0, 0, 0)
        #     time.sleep(1 / 20)
        #
        # for j in range(5):
        #     for i in range(angle + 1):
        #         self.send_attitude_target_deg(0, i, 0)
        #         time.sleep(1 / 20)
        #
        #     for i in range(angle + 1):
        #         self.send_attitude_target_deg(0, angle - i, 0)
        #         time.sleep(1 / 20)
        #
        #     for i in range(angle + 1):
        #         self.send_attitude_target_deg(0, -i, 0)
        #         time.sleep(1 / 20)
        #
        #     for i in range(angle + 1):
        #         self.send_attitude_target_deg(0, -angle + i, 0)
        #         time.sleep(1 / 20)

    def test_s_trajectory(self):
        self.logger.info("Sending")
        points = [(0.6, 1.7, 0), (0.35, 2, 0), (0, 1.7, 0), (0.15, 1.2, 0), (0.35, 1, 0),
                  (0.55, .8, 0), (0.7, 0.3, 0), (0.35, 0, 0), (0.1, 0.3, 0)]
        # points = [(0, 0.25, 0), (0, 0, 0)]

        for j in range(1):
            for point in points:
                for i in range(10):
                    if self.battery_low:
                        return
                    self.send_position_target(point[2], point[0] - 0.3, -1 - (point[1] - 1.7) / 3)
                    time.sleep(1 / 10)

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

    def send_position_estimate(self, x, y, z):
        covariance = np.array([0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0,
                               0, 0, 0, 0,
                               0, 0, 0,
                               0, 0,
                               0])
        reset_counter = 1

        self.master.mav.vision_position_estimate_send(
            int((time.time()) * 1000000),
            x,  # X y
            y,  # Y -x
            z,  # Z -z (down is negative)
            0,  # rpy_rad[0],  # Roll angle
            0,  # rpy_rad[1],  # Pitch angle
            0,  # rpy_rad[2],  # Yaw angle
            covariance,  # Row-major representation of pose 6x6 cross-covariance matrix
            reset_counter  # Estimate reset counter. Increment every time pose estimate jumps.
        )

    def run_camera_localization(self):
        position_size = 1 + 3 + 6 * 4  # 1 byte + 3 byte padding + 6 floats (4 bytes each)
        shm_name = "/pos_shared_mem"
        shm_fd = open(f"/dev/shm{shm_name}", "r+b")  # Open shared memory
        shm_map = mmap.mmap(shm_fd.fileno(), position_size, access=mmap.ACCESS_READ)

        while self.running_position_estimation:
            data = shm_map[:position_size]  # Read 28 bytes
            valid = struct.unpack("<4?", data[:4])[0]  # Extract the validity flag (1 byte)

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

                self.send_position_estimate(y, -x, -z)

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
                # q = [1.0, 0.0, 0.0, 0.0]  # identity quaternion (w, x, y, z)
                # pose_cov = [0.0] * 21
                # vel_cov = [0.0] * 21
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

    def send_vicon_position(self, x, y, z):
        self.send_position_estimate(y / 1000, x / 1000, -z / 1000)

    def send_landing_target(self, angle_x, angle_y, distance, x=0, y=0, z=0):
        """
        Sends a LANDING_TARGET MAVLink message to ArduPilot.
        """
        self.master.mav.landing_target_send(
            int((time.time() - self.start_time) * 1000000),  # time_usec: timestamp in microseconds
            0,  # target_num (not used)
            mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame: coordinate frame
            angle_x,  # angle_x: X-axis angular offset
            angle_y,  # angle_y: Y-axis angular offset
            distance,  # distance to target in meters
            0, 0,  # size_x, size_y (not used)
            x, y, z,
            0,  # q (not used)
            0,  # type (not used)
            1,  # 0 if angle_x, angle_y should be used. 1 if x, y, z fields contain position information
        )

    def start_flight(self):
        battery_thread = Thread(target=self.watch_battery, daemon=True)

        time.sleep(3)
        c.takeoff()
        time.sleep(2)

        if args.simple_takeoff:
            flight_thread = Thread(target=self.test_trajectory)
        elif args.fig8:
            flight_thread = Thread(target=self.fly_figure_eight)
        elif args.trajectory:
            time.sleep(2)
            flight_thread = Thread(target=self.send_trajectory_from_file, args=(args.trajectory,))
        else:
            time.sleep(2)
            flight_thread = Thread(target=self.test_trajectory_3)
            # flight_thread = Thread(target=self.start_mission)
            # flight_thread = Thread(target=self.test_trajectory, args=(0, 0, 0))
            # flight_thread = Thread(target=self.test_s_trajectory)
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

        if args.localize:
            self.running_position_estimation = False
            localize_thread.join()

        if args.vicon or args.save_vicon:
            vicon_thread.stop()


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
    arg_parser.add_argument("--vicon", action="store_true", help="localize using Vicon and save tracking data")
    arg_parser.add_argument("--save-vicon", action="store_true", help="save Vicon tracking data only")
    arg_parser.add_argument("--save-camera", action="store_true",
                            help="save camera at 1/10 of original fps, works with --localize")
    arg_parser.add_argument("--stream-camera", action="store_true",
                            help="stream camera at 1/10 of original fps, works with --localize")
    arg_parser.add_argument("-t", "--duration", type=float, default=15.0, help="flight duration in seconds")
    arg_parser.add_argument("--fps", type=int, default=120, help="position estimation rate, works with --localize")
    arg_parser.add_argument("--takeoff-altitude", type=float, default=1.0, help="takeoff altitude in meter")
    arg_parser.add_argument("--land-altitude", type=float, default=1.0, help="landing altitude in meter")
    arg_parser.add_argument("--voltage", type=float, default=7.4,
                            help="critical battery voltage threshold to land when reached")
    arg_parser.add_argument("--trajectory", type=str, help="path to trajectory file to follow")
    arg_parser.add_argument("--mission", type=str, help="path to mission way points file")
    arg_parser.add_argument("--simple-takeoff", action="store_true", help="takeoff and land")
    arg_parser.add_argument("--fig8", action="store_true", help="fly figure 8 pattern")
    args = arg_parser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    c = Controller(
        takeoff_altitude=args.takeoff_altitude,
        land_altitude=args.land_altitude,
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

    if args.vicon:
        from vicon import ViconWrapper

        vicon_thread = ViconWrapper(callback=c.send_vicon_position, log_level=log_level)
        vicon_thread.start()

    if args.save_vicon:
        from vicon import ViconWrapper

        vicon_thread = ViconWrapper(log_level=log_level)
        vicon_thread.start()

    if args.localize:
        localize_thread = Thread(target=c.run_camera_localization)
        lat = 12345
        lon = 12345
        alt = 0
        c.master.mav.set_gps_global_origin_send(1, lat, lon, alt)
        c.master.mav.set_home_position_send(1, lat, lon, alt, 0, 0, 0, [1, 0, 0, 0], 0, 0, 1)
        c.running_position_estimation = True
        localization_params = [
            "/home/fls/fls-marker-localization/build/eye",
            "-t", str(30 + args.duration),
            "--config", "/home/fls/fls-marker-localization/build/camera_config.json",
            "--brightness", "0.5",  # 0.01
            "--contrast", "2.5",  # 0.75
            "--exposure", "500",
            "--fps", str(args.fps),
        ]

        if args.save_camera:
            localization_params.extend(["-s", "--save-rate", "10"])
        if args.stream_camera:
            localization_params.extend(["--stream", "--stream-rate", "10"])

        c_process = subprocess.Popen(localization_params)

        time.sleep(2)
        localize_thread.start()

    if not c.set_mode('GUIDED'):
        pass
        # exit()

    if args.mission:
        c.send_mission_from_file(args.mission)

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
