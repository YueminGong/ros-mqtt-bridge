import time
import numpy as np
import rospy
import json
import paho.mqtt.client as mqtt
from std_msgs.msg import Bool, String
import threading 
from lcs_controller.msg import IRInfo, LimitSwitchInfo, LightInfo, SolenoidInfo

# MQTT Broker Configuration
broker = 'broker.emqx.io'
port = 1883
con_topic = "tmrobot/connect"
client_id = "mqttx_090f039c"
username = "tmfms"
password = "TMhk2156"

# Message Content (LCS Data)
supplier_sn = "time_medical"
external_id = "LCS01"      # need config?
eventname = "stack"

# MQTT topic LCS to LDR event
tracking_topic = f"tracking/{supplier_sn}/{external_id}"  # Tracking topic
event_topic = f"event/{supplier_sn}/{external_id}"        # Event topic

#topic FMS to lcs
robot_to_lcs_topic = f"robot_to_lcs/{supplier_sn}/{external_id}"
lcs_to_robot_ack_topic = f"lcs_to_robot_ack/{supplier_sn}/{external_id}"


# Global Variables
full_staus = 0
cage_exist = 0
clamp_status = "unclamped"
event_level = 0
event_status = "off"

clamp_bool = [0,0,0,1]
unclamp_bool = [0,0,1,0]

################# ROS Topics Part ##############################

# Function to reset cage_exist to 0 after 20 seconds
def reset_clamp_status():
    global clamp_status
    clamp_status = "unclamped"
    print("Reset cage_exist to 0 after 20 seconds")

def reset_clamp_status_inbetween():
    global clamp_status
    clamp_status = "inbetween"

# ROS event callback
def ros_callback(data, topic):
    global full_staus, cage_exist, clamp_status, event_level, event_status
    rospy.loginfo(f"Received ROS message from {topic}: {data.data}")
    if topic == "/lcs/full_state":
        full_staus = 1 if data.data else 0

    elif topic == "/lcs/cage_state":
        cage_exist = 1 if data.data else 0

    elif topic == "/lcs/ctr/solenoid":
            int_data = data.data.astype(int)
            clamp_data = np.sum(int_data[-2])
            if data.clamp_data == 1 and cage_exist == 0:  # Only trigger if cage_exist is currently 0
                clamp_status = "inbetween"
            elif clamp_data == 0 and cage_exist == 1:
                clamp_status = "clamped"
            elif clamp_data ==0 and cage_exist == 0:
                clamp_status = "unclamped"
            else: clamp_status = "error"

    elif topic == "/lcs/stuck_state":
        event_level = 2 if data.data else 0
        event_status = "on" if data.data else "off"


################# Receive MQTT message from fms ###############




################# MQTT bridge part ############################

# MQTT Connection Function
def connect_mqtt():
    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(tracking_topic)  # Subscribe to the topic after successful connection
            client.subscribe(event_topic)  # Subscribe to the topic after successful connection
        else:
            print(f"Failed to connect, return code {reason_code}")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id)

    client.username_pw_set = (username, password)

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
        "full_status": full_staus,
        "cage_exist": cage_exist,
        "clamp_status": clamp_status

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
    current_time = time.time()  # Corrected variable name
    data = {
        "supplier_sn": supplier_sn,
        "timestamp": current_time,  # Added timestamp for event
        "event": [
        {"event_name": "Stuck",
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

# Message if handling Function

def on_publish(client, userdata, mid):
    print(f"message has been published, Message ID:{mid}")

# LCS robot ans Function
def LCS_LDR_ans(client, message_id,robot_name):
    current_time = time.time()  # Corrected variable name
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
            timestamp = data.get("timestamp", 0)  # Use .get() to safely handle missing keys
            robot_name = data.get("robot_name")
            clamp_cmd = data.get("action")
            LCS_LDR_ans(client, message_id,timestamp,robot_name)

            if clamp_cmd == "lcs_clamp":
                ros_pub_clamp.publish(clamp_bool)
            if clamp_cmd == "lcs_unclamp":
                ros_pub_clamp.publish(unclamp_bool)
            else:
                client.publish(con_topic, "no event or tracking requested")
        # elif msg.topic == LDR_call_topic:
        #       if msg.mid:
        #         print(f"Message ID:{msg.mid}")
                  
        #         LCS_LDR_ans(client)
        #         payload = msg.payload.decode("utf-8")
        #         dir = json.load(payload)
        #         dir_cont = dir.get('robot_to_lcs')

        #           # ros topic pub
        #         ros_msg = Bool()
        #         ros_msg.data = msg.payload.decode()

        #         if dir_cont == "unclamp":
        #             ros_msg = [0,1,0,1]
                  
 
        #         elif dir_cont == "clamp":
        #             ros_msg = [0,1,1,0]

        #         ros_pub.publish(ros_msg)


    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")





# Main Run Function
def run():
    client = connect_mqtt()
    client.on_message = on_message  # Assign the message handler
    client.loop_start()  # Start the MQTT client loop in the background

    # Initial publish for testing (optional)
    #client.publish(topic, "hello gym!!")

    try:
        while not rospy.is_shutdown:
            LCS_tracking(client)
            LCS_event(client)

            time.sleep(3)

    except KeyboardInterrupt:
        print("Exiting program")
    finally:
        client.loop_stop()  # Properly stop the loop
        client.disconnect()  # Disconnect from the broker

# Entry Point
if __name__ == '__main__':
    rospy.init_node('mqtt_ros_bridge', anonymous=True)
    
    rospy.Subscriber("/lcs/full_state", Bool, ros_callback)  # ok
    rospy.Subscriber("/lcs/cage_state", Bool, ros_callback) 
    rospy.Subscriber("/lcs/ctr/solenoid", SolenoidInfo, ros_callback)  # ok
    rospy.Subscriber("/lcs/stuck_state", Bool, ros_callback)  # ok

    ros_pub_clamp = rospy.Publisher("/lcs/ctr/solenoid", SolenoidInfo, queue_size=10)
    # ros_pub_door = rospy.Publisher("/tmrobot/LCS_state/door", String, queue_size=10)

    run()
