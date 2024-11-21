#!/usr/bin/python3
from optparse import OptionParser
import rospy
import pymavlink
from pymavlink import mavutil
from custom_msgs.msg import commands, telemetry, esp_telemetry
from std_srvs.srv import Empty, EmptyResponse
import time
# Constants for channel mappings
# 1   Pitch
# 2   Roll
# 3   Throttle
# 4   Yaw
# 5   Forward
# 6   Lateral


class PixhawkMaster:
    """
    A class to interface with a Pixhawk via MAVLink, handle mode switching,
    arming/disarming, and send telemetry data.
    """

    def __init__(self) -> None:
        """
        Initialize the Basic class with mode, port address, and set up the MAVLink connection.
        """
        self.master_kill = True
        self.mode = options.auv_mode
        self.pixhawk_port = options.port_addr
        self.arm_state = False
        self.autonomy_switch = False
        # Initialize MAVLink connection and Mavlink msgs
        self.master = mavutil.mavlink_connection(self.pixhawk_port, baud=115200)
        self.msg_sys_status = pymavlink.dialects.v10.ardupilotmega.MAVLink_sys_status_message
        self.msg_imu = pymavlink.dialects.v10.ardupilotmega.MAVLink_scaled_imu2_message
        self.msg_attitude = pymavlink.dialects.v10.ardupilotmega.MAVLink_attitude_quaternion_message
        self.msg_vfr_hud = pymavlink.dialects.v10.ardupilotmega.MAVLink_vfr_hud_message
        self.msg_depth = pymavlink.dialects.v10.ardupilotmega.MAVLink_scaled_pressure2_message
        self.thruster_pwms = [1500]*8  
        # ROS subscriber and publisher
        self.thruster_subs = rospy.Subscriber(
            "/master/commands", commands, self.__callback__, queue_size=1
        )
        self.telemetry_pub = rospy.Publisher(
            "/master/telemetry", telemetry, queue_size=1
        )
        self.thruster_subs_rov = rospy.Subscriber(
            "/rov/commands", commands, self.rov_callback, queue_size=1
        )
        self.kill_sub = rospy.Subscriber(
            "/esp/telemetry", esp_telemetry, self.kill_callback, queue_size=1
        )
        self.autonomy_service = rospy.Service("/mira/switch", Empty, self.service_callback)
        self.channel_ary = [1500] * 8  # Initialize channel values array
        self.master.wait_heartbeat()  # Wait for the heartbeat from the Pixhawk
        self.telem_msg = telemetry()  # Initialize telemetry message

    def kill_callback(self, msg):
        if (msg.kill_switch == True):
            self.disarm()
            rospy.logwarn("KILL SWITCH ENABLED, DISARMING AND KILLING")
            exit()
                          

    def service_callback(self, msg):
        self.autonomy_switch = not self.autonomy_switch
        if (self.autonomy_switch==True):
            rospy.loginfo("AUTONOMY MODE")
        else:
            rospy.loginfo("ROV MODE")
        return EmptyResponse()

    def rov_callback(self, msg):
        if msg.arm == 1 and self.arm_state == False:
                self.arm()
                self.arm_state = True
        elif msg.arm == 0 and self.arm_state == True:
                self.disarm()
                self.arm_state = False

        if self.autonomy_switch==False:
            self.channel_ary[0] = msg.pitch
            self.channel_ary[1] = msg.roll
            self.channel_ary[2] = msg.thrust
            self.channel_ary[3] = msg.yaw
            self.channel_ary[4] = msg.forward
            self.channel_ary[5] = msg.lateral
            self.channel_ary[6] = msg.servo1
            self.channel_ary[7] = msg.servo2

            # Handle mode switching
            if self.mode != msg.mode:
                if self.arm_state == False:
                    self.mode = msg.mode
                    self.mode_switch()
                else:
                    rospy.logwarn("Disarm Pixhawk to change modes.")

    def __callback__(self, msg):
        """
        Callback function to handle incoming commands and update RC channel values.
        Also handles arming/disarming and mode switching.
        """
        if self.autonomy_switch==True:
            self.channel_ary[0] = msg.pitch
            self.channel_ary[1] = msg.roll
            self.channel_ary[2] = msg.thrust
            self.channel_ary[3] = msg.yaw
            self.channel_ary[4] = msg.forward
            self.channel_ary[5] = msg.lateral
            self.channel_ary[6] = msg.servo1
            self.channel_ary[7] = msg.servo2

            # Handle arming/disarming
            if msg.arm == 1 and self.arm_state == False:
                self.arm()
                self.arm_state = True
            elif msg.arm == 0 and self.arm_state == True:
                self.disarm()
                self.arm_state = False

            # Handle mode switching
            if self.mode != msg.mode:
                if self.arm_state == False:
                    self.mode = msg.mode
                    self.mode_switch()
                else:
                    rospy.logwarn("Disarm Pixhawk to change modes.")

    def arm(self):
        """
        Send an arm command to the Pixhawk.
        """
        self.master.wait_heartbeat()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # Arm
            0,
            0,
            0,
            0,
            0,
            0,
        )
        rospy.loginfo("Arm command sent to Pixhawk")

    def disarm(self):
        """
        Send a disarm command to the Pixhawk.
        """
        self.master.wait_heartbeat()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # Disarm
            0,
            0,
            0,
            0,
            0,
            0,
        )
        rospy.loginfo("Disarm command sent to Pixhawk")

    def mode_switch(self):
        """
        Switch the Pixhawk mode.
        """
        if self.mode not in self.master.mode_mapping():
            rospy.logerr(f"Unknown mode: {self.mode}")
            rospy.loginfo(f"Try: {list(self.master.mode_mapping().keys())}")
            exit(1)

        mode_id = self.master.mode_mapping()[self.mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        rospy.loginfo(f"Mode changed to: {self.mode}")

    def set_rc_channel_pwm(self, id, pwm):
        """
        Set the PWM value for a specified RC channel.
        """
        if id < 1:
            rospy.logwarn("Channel does not exist.")
            return

        if id < 9:
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[id - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_channel_values,
            )  # RC channel list, in microseconds

    def actuate(self):
        """
        Send RC channel commands to the Pixhawk based on updated channel values.
        """
        self.set_rc_channel_pwm(1, int(self.channel_ary[0]))
        self.set_rc_channel_pwm(2, int(self.channel_ary[1]))
        self.set_rc_channel_pwm(3, int(self.channel_ary[2]))
        self.set_rc_channel_pwm(4, int(self.channel_ary[3]))
        self.set_rc_channel_pwm(5, int(self.channel_ary[4]))
        self.set_rc_channel_pwm(6, int(self.channel_ary[5]))
        self.set_rc_channel_pwm(7, int(self.channel_ary[6]))
        self.set_rc_channel_pwm(8, int(self.channel_ary[7]))

    def request_message_interval(self, message_id: int, frequency_hz: float):
        """
        Request the interval at which a specified message should be sent.
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,  # The MAVLink message ID
            1e6 / frequency_hz,  # The interval between two messages in microseconds
            0,
            0,
            0,
            0,
            0,
        )
        response = self.master.recv_match(type="COMMAND_ACK", blocking=True)
        if (
            response
            and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
            and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        ):
            rospy.loginfo("Command Accepted")
        else:
            rospy.logerr("Command Failed")

    def telem_publish_func(self, timestamp_passed):
        """
        Publish telemetry data based on received MAVLink messages.
        """

        self.telem_msg.battery_voltage = (self.msg_sys_status.voltage_battery)/1000
        self.telem_msg.timestamp = timestamp_passed
        self.telem_msg.internal_pressure = self.msg_vfr_hud.alt
        self.telem_msg.external_pressure = self.msg_depth.press_abs
        self.telem_msg.heading = self.msg_vfr_hud.heading
        self.telem_msg.imu_gyro_x = self.msg_imu.xgyro
        self.telem_msg.imu_gyro_y = self.msg_imu.ygyro
        self.telem_msg.imu_gyro_z = self.msg_imu.zgyro
        self.telem_msg.imu_gyro_compass_x = self.msg_imu.xmag
        self.telem_msg.imu_gyro_compass_y = self.msg_imu.ymag
        self.telem_msg.q1 = self.msg_attitude.q1
        self.telem_msg.q2 = self.msg_attitude.q2
        self.telem_msg.q3 = self.msg_attitude.q3
        self.telem_msg.q4 = self.msg_attitude.q4
        self.telem_msg.rollspeed = self.msg_attitude.rollspeed
        self.telem_msg.pitchspeed = self.msg_attitude.pitchspeed
        self.telem_msg.yawspeed = self.msg_attitude.yawspeed
        self.telem_msg.thruster_pwms[0] = self.thruster_pwms[0]
        self.telem_msg.thruster_pwms[1] = self.thruster_pwms[1]
        self.telem_msg.thruster_pwms[2] = self.thruster_pwms[2]
        self.telem_msg.thruster_pwms[3] = self.thruster_pwms[3]
        self.telem_msg.thruster_pwms[4] = self.thruster_pwms[4]
        self.telem_msg.thruster_pwms[5] = self.thruster_pwms[5]
        self.telem_msg.thruster_pwms[6] = self.thruster_pwms[6]
        self.telem_msg.thruster_pwms[7] = self.thruster_pwms[7]

        if((self.telem_msg.battery_voltage)<15):
#            rospy.logwarn(f"Battery Critically Low: {self.telem_msg.battery_voltage}V")
            pass
        self.telemetry_pub.publish(self.telem_msg)




if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("pymav_master", anonymous=True)#, disable_signals=True)

    # Command line options
    parser = OptionParser(description="description for prog")
    parser.add_option(
        "-p",
        "--port",
        dest="port_addr",
        default="/dev/Pixhawk",
        help="Pass Pixhawk Port Address",
        metavar="VAR",
    )
    parser.add_option(
        "-m",
        "--mode",
        dest="auv_mode",
        default="STABILIZE",
        help="Pass Pixhawk Mode",
        metavar="VAR",
    )
    (options, args) = parser.parse_args()

    # Instantiate class
    obj = PixhawkMaster()

    # Request message intervals
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 300)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 300)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 300)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 300)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2, 300)

    # Main loop
    while not rospy.is_shutdown():
        obj.actuate()
        try:

            msg = obj.master.recv_match()
            timestamp_now = rospy.get_time()
            print(timestamp_now)
            #if msg:
                #rospy.loginfo(f"Received message of type {msg.get_type()}")
                
            if not msg:
                continue
            if msg.get_type() == 'SYS_STATUS':
                obj.msg_sys_status =  msg
                #rospy.loginfo("SYS")
                obj.telem_publish_func(timestamp_now)
                
            elif msg.get_type() == 'SCALED_IMU2':
                obj.msg_imu = msg
                #rospy.loginfo("IMU")
                obj.telem_publish_func(timestamp_now)

            elif msg.get_type() == 'ATTITUDE_QUATERNION':
                obj.msg_attitude = msg
                #rospy.loginfo("QUAT")
                obj.telem_publish_func(timestamp_now)
            
            elif msg.get_type() == 'VFR_HUD':
                obj.msg_vfr_hud = msg
                #rospy.loginfo("HUD")
                obj.telem_publish_func(timestamp_now)
            
            elif msg.get_type() == 'SCALED_PRESSURE2':
                obj.msg_depth = msg
                #rospy.loginfo("PRESSURE")
                obj.telem_publish_func(timestamp_now)

            elif msg.get_type() == 'SERVO_OUTPUT_RAW':
                obj.servo_pwms = msg
                #rospy.loginfo("Servo")
                obj.telem_publish_func(timestamp_now)
        
        except Exception as e:
            rospy.logwarn(f"Error receiving message: {e}")
            continue

