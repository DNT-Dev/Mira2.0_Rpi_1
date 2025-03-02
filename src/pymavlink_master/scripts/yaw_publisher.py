#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32


def main():
    """
    Initializes the ROS node, sets up the serial connection, and publishes yaw data
    from an Arduino to a ROS topic.
    """
    rospy.init_node("arduino_yaw_publisher", anonymous=True)
    pub = rospy.Publisher("mira/heading", Float32, queue_size=10)
    rate = rospy.Rate(100)

    # Parameters for serial port and baud rate
    serial_port = rospy.get_param("~serial_port", "/dev/Mega")
    serial_baud = rospy.get_param("~serial_baud", 115200)

    # Establish serial connection
    try:
        ser = serial.Serial(serial_port, serial_baud)
        rospy.loginfo(f"Connected to {serial_port} at {serial_baud} baud.")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to connect to {serial_port} at {serial_baud} baud: {e}")
        return
    line = None
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode("utf-8").strip()
                yaw_value = float(line)
                pub.publish(yaw_value)
                rospy.loginfo(f"Published Yaw value: {yaw_value}")
            except ValueError as e:
                rospy.logwarn(f"ValueError: {e} with line '{line}'")
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Yaw Publisher Node Terminated.")
