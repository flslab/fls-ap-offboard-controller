from pymavlink import mavutil
import time

# Serial connection parameters
device = "/dev/serial0"
baudrate = 115200  # Change this if needed

# Connect to the flight controller
master = mavutil.mavlink_connection(device, baud=baudrate)

# Wait for a heartbeat before sending commands
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

# Set mode to GUIDED
def set_guided_mode():
    mode = 'GUIDED'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    ack = False
    while not ack:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            print("GUIDED mode set")
            ack = True

# Arm the drone
def arm():
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Drone armed")

# Take off to target altitude (in meters)
def takeoff(target_altitude):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        target_altitude
    )
    print(f"Takeoff command sent to {target_altitude} meters")

# Land the drone
def land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("Landing command sent")

# Main sequence
set_guided_mode()
arm()
takeoff(1.0)

# Wait until near 1 meter (or timeout)
print("Waiting to reach target altitude...")
start_time = time.time()
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        alt = msg.relative_alt / 1000.0  # mm to meters
        print(f"Altitude: {alt:.2f}m")
        if alt > 0.95:
            print("Target altitude reached")
            break
    if time.time() - start_time > 20:
        print("Timeout while waiting for altitude")
        break

# Wait a bit before landing
time.sleep(3)
land()
print("Mission complete.")