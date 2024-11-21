"""
Example: Set that a message is streamed at particular rate
"""

from pymavlink import mavutil

# Start a connection listening on a UDP port
connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait for the first heartbeat to set the system and component ID of remote system for the link
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))


def request_message_interval(message_id: int, frequency_hz: float):
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate. pure value is in microseconds
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )
    response = connection.recv_match(type='COMMAND_ACK', blocking=True)
    if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
    else:
        print("Command failed")


# message_id_array ={mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU} 
# request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, 10)
# request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 10)
# request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, 10)
# request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 10)

request_message_interval(24, 10)




# # Define command_long_encode message to send MAV_CMD_SET_MESSAGE_INTERVAL command
# # param1: MAVLINK_MSG_ID_BATTERY_STATUS (message to stream)
# # param2: 1000000 (Stream interval in microseconds)
# message = connection.mav.command_long_encode(
#         connection.target_system,  # Target system ID
#         connection.target_component,  # Target component ID
#         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
#         0,  # Confirmation
#         mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,  # param1: Message ID to be streamed
#         10, # param2: Interval in microseconds
#         0,       # param3 (unused)
#         0,       # param4 (unused)
#         0,       # param5 (unused)
#         0,       # param5 (unused)
#         0        # param6 (unused)
#         )

# # Send the COMMAND_LONG     
# connection.mav.send(message)

# # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result


while True:  
    try:  
        msg = connection.recv_match(type='GPS_RAW_INT',blocking=True)
        # msg_imu = connection.recv_match(type='SCALED_IMU',blocking=True)
        # msg_attitude = connection.recv_match(type='ATTITUDE_QUATERNION',blocking=True)
        # rospy.init_node('PyMavMasterNode')
        # msg_vfr_hud = connection.recv_match(type='VFR_HUD',blocking=True)
        # msg_depth = connection.recv_match(type='SCALED_PRESSURE2',blocking=True)
        # msg_depth = connection.recv_match(type='SCALED_IMU2',blocking=True)
        # print("\n\n\n")
        # print("------------------------------------------------------------------------------")
        # print(msg_imu)
        # print("------------------------------------------------------------------------------")
        # print(msg_attitude)
        # print("------------------------------------------------------------------------------")
        # print(msg_vfr_hud)
        # print("------------------------------------------------------------------------------")


#request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 10)
        print(msg)
        #print(msg.get_type())
    except:
        print('No message recieved')
# try: 
#     altitude = connection.messages['MAVLINK_MSG_ID_SCALED_IMU'].alt  # Note, you can access message fields as attributes!
#     timestamp = connection.time_since('MAVLINK_MSG_ID_SCALED_IMU')
#     print(altitude,timestamp)
# except:
#     print('No GPS_RAW_INT message received')
