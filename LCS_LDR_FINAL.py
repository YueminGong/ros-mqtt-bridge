#!/usr/bin/env python3
import time
import numpy as np
import rospy
import json
import random
import paho.mqtt.client as mqtt
from std_msgs.msg import Bool, String
import threading
from lcs_controller.msg import IRInfo, LimitSwitchInfo, LightInfo, SolenoidInfo

# MQTT Broker Configuration
broker = '10.1.3.1'
port = 1883
con_topic = f"tmrobot/connect"
client_id = f'publish-{random.randint(0, 1000)}'
username = "tmfms"
password = "TMhk2156"

# Message Content (LCS Data)
supplier_sn = "time_medical"
external_id = "LCS09"      # need config?
eventname = "stack"

# MQTT topic LCS to LDR event
tracking_topic = f"tracking/{supplier_sn}/{external_id}"  # Tracking topic
event_topic = f"event/{supplier_sn}/{external_id}"        # Event topic

# topic FMS to lcs
robot_to_lcs_topic = f"robot_to_lcs/{supplier_sn}/{external_id}"
lcs_to_robot_ack_topic = f"lcs_to_robot_ack/{supplier_sn}/{external_id}"

# Global Variables
full_status = 0
cage_exist = 0
clamp_status = "unclamped"
event_level = 2
event_status = "off"


clamp_bool = [0, 0, 0, 1]
unclamp_bool = [0, 0, 1, 0]

################# ROS Topics Part ##############################

# Function to reset cage_exist to 0 after 20 seconds
def reset_clamp_status():
    global clamp_status
    clamp_status = "unclamped"
    print("Reset cage_exist to 0 after 20 seconds")

def reset_clamp_status_inbetween():
    global clamp_status
    clamp_status = "inbetween"

# ROS event callback functions
def full_state_callback(data):
    global full_status
    full_status = 1 if data.data else 0
    rospy.loginfo(f"Received ROS message from /lcs/full_state: {data.data}")

def limit_switch_callback(data):
    global cage_exist
    cage_exist_data = np.array(data.ls_states).astype(int)
    cage_exist_en = cage_exist_data[-1]
    cage_exist = 0 if cage_exist_en else 1             # check the confi
    rospy.loginfo(f"Received ROS message from /lcs/state/ls: {data.ls_states}")

def solenoid_callback(data):
    global clamp_status
    int_data = np.array(data.solenoid_ctr).astype(int)
    clamp_data = np.sum(int_data[-2])
    if clamp_data == 1 and cage_exist == 0:
        clamp_status = "inbetween"
    elif clamp_data == 0 and cage_exist == 1:
        clamp_status = "clamped"
    elif clamp_data == 0 and cage_exist == 0:
        clamp_status = "unclamped"
    else:
        clamp_status = "error"
    rospy.loginfo(f"Received ROS message from /lcs/ctr/solenoid: {data.solenoid_ctr}")

def stuck_state_callback(data):
    global event_level, event_status
    event_level = 3 if data.data else 2
    event_status = "on" if data.data else "off"
    rospy.loginfo(f"Received ROS message from /lcs/stuck_state: {data.data}")

################# MQTT bridge part ############################

# MQTT Connection Function
def connect_mqtt():
    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(tracking_topic)  # Subscribe to the topic after successful connection
            client.subscribe(event_topic)  # Subscribe to the topic after successful connection
            client.subscribe(robot_to_lcs_topic)     #Subscribe to the topic after successful connection
        else:
            print(f"Failed to connect, return code {reason_code}")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id)
    client.username_pw_set(username, password)  # Corrected method call
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

# LCS Tracking Data Publishing Function
def LCS_tracking(client):
    current_time = time.time()
    data = {
        "supplier_sn": supplier_sn,
        "external_id": external_id,
        "timestamp": current_time,
        "full_status": full_status,
        "cage_exist": cage_exist,
        "clamp_status": clamp_status,
        "event": [
            {
                "event_name": "Stuck",
                "event_value": "",
                "event_level": event_level,
                "event_status": event_status,
            }
        ]

            }
    msg = json.dumps(data)
    result = client.publish(tracking_topic, msg)  # Capture the publish result
    status = result[0]
    if status == 0:
        print(f"Sent tracking data: `{msg}` to topic `{tracking_topic}`")
    else:
        print(f"Failed to send tracking data to topic {tracking_topic}")

# LCS Event Data Publishing Function
def LCS_event(client):
    global event_status, event_level
    current_time = time.time()
    data = {
        "supplier_sn": supplier_sn,
        "timestamp": current_time,  # Added timestamp for event
        "event": [
            {
                "event_name": "Stuck",
                "event_value": "",
                "event_level": event_level,
                "event_status": event_status,
            }
        ]
    }
    msg = json.dumps(data)
    result = client.publish(event_topic, msg)  # Capture the publish result
    status = result[0]
    if status == 0:
        print(f"Sent event data: `{msg}` to topic `{event_topic}`")
    else:
        print(f"Failed to send event data to topic {event_topic}")

# LCS robot ans Function
def LCS_LDR_ans(client, message_id, robot_name):
    current_time = time.time()
    data = {
        "message_id": message_id,
        "timestamp": current_time,
        "robot_name": robot_name
    }
    msg = json.dumps(data)
    result = client.publish(lcs_to_robot_ack_topic, msg)  # Capture the publish result
    status = result[0]
    if status == 0:
        print(f"Sent event data: `{msg}` to topic `{lcs_to_robot_ack_topic}`")
    else:
        print(f"Failed to send event data to topic {lcs_to_robot_ack_topic}")

# Message Handling Function (called when a message is received)
def on_message(client, userdata, msg):
    try:
        if msg.topic == robot_to_lcs_topic:
            payload = msg.payload.decode("utf-8")
            print(f"Received message: {payload} from topic: {msg.topic}")
            data = json.loads(payload)
            message_id = data.get("message_id", 0)  # Use .get() to safely handle missing keys
            robot_name = data.get("robot_name")
            clamp_cmd = data.get("action")
            LCS_LDR_ans(client, message_id, robot_name)
            client.publish(con_topic, "clamp cmd requested")

            if clamp_cmd == "lcs_clamp":
                solenoid_msg = SolenoidInfo()
                solenoid_msg.data = clamp_bool
                ros_pub_clamp.publish(solenoid_msg)
            elif clamp_cmd == "lcs_unclamp":
                solenoid_msg = SolenoidInfo()
                solenoid_msg.data = unclamp_bool
                ros_pub_clamp.publish(solenoid_msg)
        else:
            client.publish(con_topic, "no clamp cmd requested")
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

# Main Run Function
def run():
    client = connect_mqtt()
    client.on_message = on_message  # Assign the message handler
    client.loop_start()  # Start the MQTT client loop in the background

    try:
        while not rospy.is_shutdown():
            LCS_tracking(client) 
            #LCS_event(client) # lcs tracking includ event content
            time.sleep(3)
    except KeyboardInterrupt:
        print("Exiting program")
    finally:
        client.loop_stop()  # Properly stop the loop
        client.disconnect()  # Disconnect from the broker

# Entry Point
if __name__ == '__main__':
    rospy.init_node('mqtt_ros_bridge', anonymous=True)

    # ROS Subscribers
    rospy.Subscriber("/lcs/full_state", Bool, full_state_callback)
    rospy.Subscriber("/lcs/state/ls", LimitSwitchInfo, limit_switch_callback)
    rospy.Subscriber("/lcs/ctr/solenoid", SolenoidInfo, solenoid_callback)
    rospy.Subscriber("/lcs/stuck_state", Bool, stuck_state_callback)

    # ROS Publishers
    ros_pub_clamp = rospy.Publisher("/lcs/ctr/solenoid", SolenoidInfo, queue_size=10)

    run()
