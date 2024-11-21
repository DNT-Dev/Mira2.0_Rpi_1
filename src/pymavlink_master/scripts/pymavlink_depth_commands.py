#!/usr/bin/python3
import rospy
from pymavlink import mavutil
from optparse import OptionParser
from custom_msgs.msg import commands 
from custom_msgs.msg import telemetry
import time

# 1 	Pitch
# 2 	Roll
# 3 	Throttle
# 4 	Yaw
# 5 	Forward
# 6 	Lateral

class Basic:

    def __init__(self) -> None:  
        self.mode = options.auv_mode
        self.pixhawk_port = options.port_addr
        self.arm_state= False            
        self.master = mavutil.mavlink_connection(self.pixhawk_port, baud=115200)
        self.thruster_subs = rospy.Subscriber("/master/commands", commands, self.callback, queue_size=1)
        self.telemetry_pub = rospy.Publisher('/master/telemetry', telemetry, queue_size=1)
        self.channel_ary=[1500]*8
        self.master.wait_heartbeat()
        self.telem_msg = telemetry()
        
    def callback(self, msg):
        self.channel_ary[0] = msg.pitch 
        self.channel_ary[1] = msg.roll
        self.channel_ary[2] = msg.thrust
        self.channel_ary[3] = msg.yaw
        self.channel_ary[4] = msg.forward
        self.channel_ary[5] = msg.lateral
        self.channel_ary[6] = msg.servo1
        self.channel_ary[7] = msg.servo2

        if (msg.arm == 1 and self.arm_state==False):
            self.arm()
            self.arm_state = True
            
        elif(msg.arm == 0 and self.arm_state==True):
            self.disarm()
            self.arm_state = False

        
        if (self.mode!=msg.mode):
            if(self.arm_state == False):
                self.mode=msg.mode
                self.mode_switch()
            else:
                print("Need to disarm in order to change mode")

    def arm(self):
        self.master.wait_heartbeat()
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,1, 0, 0, 0, 0, 0, 0)
        print("arm command sent to pix")
    
    def disarm(self):
        self.master.wait_heartbeat()
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,0, 0, 0, 0, 0, 0, 0)
        print("sent disarm")

    def mode_switch(self):
        if self.mode == 'ALT_HOLD':
            boot_time = time.time()
            for i in range(3):
                self.set_depth(-3,boot_time)
        elif self.mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(self.mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            exit(1)
        mode_id = self.master.mode_mapping()[self.mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        print("mode changed to:" ,(self.mode))
        
    def set_rc_channel_pwm(self, id, pwm):
        if id < 1:
            print("Channel does not exist.")
            return
        
        if id < 9:
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[id - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,                # target_system
                self.master.target_component,             # target_component
                *rc_channel_values)                  # RC channel list, in microseconds.

    def actuate(self):
        self.set_rc_channel_pwm(1, int(self.channel_ary[0]))
        self.set_rc_channel_pwm(2, int(self.channel_ary[1]))
        self.set_rc_channel_pwm(3, int(self.channel_ary[2]))
        self.set_rc_channel_pwm(4, int(self.channel_ary[3]))
        self.set_rc_channel_pwm(5, int(self.channel_ary[4]))
        self.set_rc_channel_pwm(6, int(self.channel_ary[5]))
        self.set_rc_channel_pwm(7, int(self.channel_ary[6]))
        self.set_rc_channel_pwm(8, int(self.channel_ary[7]))

    def request_message_interval(self, message_id: int, frequency_hz: float):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate. pure value is in microseconds
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )
        response = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Command accepted")
        else:
            print("Command failed")
                
    def telem_publish_func(self, imu_msg, attitude_msg, vfr_hud_msg, depth_msg):
        self.telem_msg.timestamp=imu_msg.time_boot_ms
        self.telem_msg.internal_pressure=vfr_hud_msg.alt
        self.telem_msg.external_pressure=depth_msg.press_abs
        self.telem_msg.heading=vfr_hud_msg.heading
        self.telem_msg.imu_gyro_x=imu_msg.xgyro
        self.telem_msg.imu_gyro_y=imu_msg.ygyro
        self.telem_msg.imu_gyro_z=imu_msg.zgyro
        self.telem_msg.imu_gyro_compass_x=imu_msg.xmag
        self.telem_msg.imu_gyro_compass_y=imu_msg.ymag
        self.telem_msg.imu_gyro_compass_z=imu_msg.zmag
        self.telem_msg.q1=attitude_msg.q1
        self.telem_msg.q2=attitude_msg.q2
        self.telem_msg.q3=attitude_msg.q3
        self.telem_msg.q4=attitude_msg.q4
        self.telem_msg.rollspeed=attitude_msg.rollspeed
        self.telem_msg.pitchspeed=attitude_msg.pitchspeed
        self.telem_msg.yawspeed=attitude_msg.yawspeed
        self.telemetry_pub.publish(self.telem_msg)


    def set_depth(self,depth,boot_time):
        #ALT_HOLD_MODE = self.master.mode_mapping()['ALT_HOLD']
        #while not self.master.wait_heartbeat().custom_mode == ALT_HOLD_MODE:
        #    self.master.set_mode('ALT_HOLD')

        self.master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        self.master.target_system, self.master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=(
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ),
        lat_int=0, lon_int=0, alt = depth,  # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0,  # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
        )

if __name__ == "__main__":
    rospy.init_node('pymav_master', anonymous=True)
    
    parser = OptionParser(description='description for prog')
    parser.add_option("-p", "--port", dest="port_addr",default="/dev/ttyACM0",help="Pass Pixhawk Port Address", metavar="VAR")
    parser.add_option("-m", "--mode", dest="auv_mode", default='STABILIZE',help="Pass Pixhawk Mode", metavar="VAR")
    (options, args) = parser.parse_args()
    obj = Basic()

    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2000)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,100)

    while not rospy.is_shutdown():
        obj.actuate()
        try:  
            msg_imu = obj.master.recv_match(type='SCALED_IMU',blocking=True)
            msg_attitude = obj.master.recv_match(type='ATTITUDE_QUATERNION',blocking=True)
            msg_vfr_hud = obj.master.recv_match(type='VFR_HUD',blocking=True)
            msg_depth = obj.master.recv_match(type='SCALED_PRESSURE2',blocking=True)
        except:
            print('No message recieved')
        obj.telem_publish_func(msg_imu,msg_attitude,msg_vfr_hud,msg_depth)

        
        
    # rospy.spin()
