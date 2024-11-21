#!/usr/bin/python3
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import Int32
broker_address = "192.168.214.112"  # Replace with the broker address
port = 1883
topic = "compass"
rospy.init_node("yaw_on_phone_pub")
pu = rospy.Publisher("/mira/yaw/mobile", Int32, queue_size=10) 
def on_message(client, userdata, message):
    print("Received compass data:", message.payload.decode())
    i = Int32()
    i.data = int(message.payload.decode())
    pu.publish(i)
client = mqtt.Client()
client.on_message = on_message
client.connect(broker_address, port)
client.su:q
