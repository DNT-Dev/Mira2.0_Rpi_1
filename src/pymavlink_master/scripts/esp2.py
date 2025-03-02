#!/usr/bin/python3


import serial

def main():
    # rospy.init_node('esp_telemetry_node')  # Initialize the node
    # pub = rospy.Publisher('/esp/telemetry', esp_telemetry, queue_size=10)

    source = serial.Serial('/dev/esp8266', 115200, timeout=1)  # Initialize the serial connection

    # telemetry_msg = esp_telemetry()  # Create a message object

    try:
        while not False:
            print("Reading from ESP")
            data = source.readline()
            try:
                Data=int(data.decode('utf').rstrip('\n'))
                if Data==1:
                    print(True)
                    # telemetry_msg.kill_switch = True
                else:
                    print(False)
                    # telemetry_msg.kill_switch = False
                # pub.publish(telemetry_msg)
            except Exception as e:
                print(e)
                continue
    except KeyboardInterrupt:
        pass
        # rospy.loginfo("Shutting down ESP Telemetry Node")

if __name__ == '__main__':
    main()               
