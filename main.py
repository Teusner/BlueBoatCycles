import time
# Import mavutil
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import numpy as np

# Connect to default BlueOS mavlink port
print('Waiting to connect..')
master = mavutil.mavlink_connection('tcp:192.168.2.2:5777', source_system=2)
boot_time = time.time()

print('Waiting for vehicle..')
master.wait_heartbeat()

def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

# Configure ATTITUDE message to be sent at 2Hz
#request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)

def set_target_attitude(roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in radians

    """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([angle for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )

yaw_d = 0
Kp = 0.1

master.arducopter_arm()
master.motors_armed_wait()

print(master.target_system)
print(master.target_component)

master.mav.set_mode_send(master.target_system, mavlink1.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)


t0 = time.time()
cnt = 0

times = [10, 10]
yaws = [0, np.pi/2]

while True:
    if time.time()-t0 > times[cnt]:
        t0 = time.time()
        cnt = (cnt+1)%2
        print(f"New state yaw={yaws[cnt]}")
    set_target_attitude(0, 0, yaws[cnt])
    time.sleep(0.1)