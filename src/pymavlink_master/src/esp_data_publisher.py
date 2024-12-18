#!/usr/bin/python3

import rospy
from custom_msgs.msg import esp_telemetry
import serial

def main():
    rospy.init_node('esp_telemetry_node')  # Initialize the node
    pub = rospy.Publisher('/esp/telemetry', esp_telemetry, queue_size=10)

    source = serial.Serial('/dev/esp8266', 115200, timeout=1)  # Initialize the serial connection

    telemetry_msg = esp_telemetry()  # Create a message object

    try:
        while not rospy.is_shutdown():
            data = source.readline()
            try:
                Data=int(data.decode('utf').rstrip('\n'))
                if Data==1:
                    telemetry_msg.kill_switch = False
                else:
                    telemetry_msg.kill_switch = True
                pub.publish(telemetry_msg)
            except ValueError:
                continue
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down ESP Telemetry Node")

if __name__ == '__main__':
    main()               
