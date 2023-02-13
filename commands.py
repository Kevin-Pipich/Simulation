from pymavlink import mavutil

def takeoff(master, min_pitch, yaw, lat, long, alt):
    command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    master.mav.command_long_send(master.target_system, master.target_component, command, 0, min_pitch, 0, 0, yaw, lat, long, alt)

def arm_disarm(master, arm=True):
    command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    master.mav.command_long_send(master.target_system, master.target_component, command, 0, arm, 0, 0, 0, 0, 0, 0)

def set_position(master, type_mask, x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate):
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    type_mask, x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate))