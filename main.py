from pymavlink import mavutil
import numpy as np
import commands as cmd

master = mavutil.mavlink_connection('udpin:localhost:14540')

master.target_system = 1
master.target_component = 0

master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# cmd.arm_disarm(master, True)

# cmd.takeoff(master, 2, 0, 0.1, 0, 1000)

cmd.set_position(master, np.uint16(0), 20, 0, -100, 10, 0, 10, 0, 0, 0, 0, 0)

while 1:
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    print(msg)