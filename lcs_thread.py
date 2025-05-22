#!/usr/bin/env python3
import time
import numpy as np
import rospy
import json
import random
import paho.mqtt.client as mqtt
from std_msgs.msg import Bool, String, Float32
import threading
from lcs_controller.msg import IRInfo, LimitSwitchInfo, LightInfo, SolenoidInfo

# MQTT Broker??
broker = '10.1.6.233'
port = 1883
con_topic = f"tmrobot/connect"
client_id = f'publish-{random.randint(0, 1000)}'
username = "tmfms"
password = "TMhk2156"

# ????
supplier_sn = "time_medical"
external_id = "LCS01"
eventname = "stack"

# MQTT????
tracking_topic = f"tracking_lcs/{supplier_sn}/{external_id}"
event_topic = f"event/{supplier_sn}/{external_id}"
robot_to_lcs_topic = f"robot_to_lcs/{supplier_sn}/{external_id}"
lcs_to_robot_ack_topic = f"lcs_to_robot_ack/{supplier_sn}/{external_id}"
fms_to_robot_topic = f"device_to_lcs_door/{supplier_sn}/{external_id}"
lcs_to_fms_ack_topic = f"lcs_door_to_device/{supplier_sn}/{external_id}"

# ??????
full_status = -1
cage_exist = 0
clamp_status = "unclamped"
event_level = 2
event_status = "off"
full_status_high = 0.25
door_state = -1  # -1:??, 0:??, 1:??

# ???????
current_door_cmd = None     # ????????
door_operation_start = 0    # ???????
door_operation_time = 25    # ?????(?)
door_operation_running = False  # ?????????

# ????????
clamp_bool = [0, 0, 0, 1]
unclamp_bool = [0, 0, 1, 0]
doorclose_bool = [1, 0, 0, 0]
dooropen_bool = [0, 1, 0, 0]
waitting_bool = [0, 0, 0, 0]

# ??:?
last_door_state = door_state
last_event_level = event_level
last_event_status = event_status

################# ROS???? ##############################


# ROS??????
def full_state_callback(data):
    global full_status
    full_status = 1 if data.data else 0
    rospy.loginfo(f"Received ROS message from /lcs/full_state: {data.data}")

def full_state_high_callback(data):
    global full_status_high
    full_status_high = data.data
    rospy.loginfo(f"Received ROS message from /lcs/full_high_state: {data.data}")

def limit_switch_callback(data):
    global cage_exist
 
    cage_exist_data = np.array(data.ls_states).astype(int)
    cage_exist_en = cage_exist_data[-2]
    cage_exist_old = cage_exist
    cage_exist = 1 if cage_exist_en == 0 else 0



def solenoid_callback(data):
    global clamp_status
    int_data = np.array(data.solenoid_ctr).astype(int)
    clamp_data = int_data[-2]
    inbetween_data = np.sum(int_data[-2:])
    
    if inbetween_data == 1:
        clamp_status = "inbetween"
    elif clamp_data == 0 and cage_exist == 1:
        clamp_status = "clamped"
    elif clamp_data == 0 and cage_exist == 0:
        clamp_status = "unclamped"
    else:
        clamp_status = "error"


def stuck_state_callback(data):
    global event_level, event_status, last_event_level, last_event_status
    last_event_level = event_level
    last_event_status = event_status
    
    event_level = 3 if data.data else 2
    event_status = "on" if data.data else "off"
    
    # ?????????MQTT
    if last_event_level != event_level or last_event_status != event_status:
        rospy.loginfo(f"Event status updated: level={event_level}, status={event_status}")



################# MQTT???? ############################

# MQTT????
def connect_mqtt():
    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(tracking_topic)
            client.subscribe(event_topic)
            client.subscribe(robot_to_lcs_topic)
            client.subscribe(fms_to_robot_topic)
        else:
            print(f"Failed to connect, return code {reason_code}")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

# ????????
def LCS_tracking(client):
    global door_state, current_door_cmd, clamp_status
    current_time = time.time()
    data = {
        "supplier_sn": supplier_sn,
        "external_id": external_id,
        "timestamp": current_time,
        "full_status_high": full_status_high,
        "cage_exist": cage_exist,
        "clamp_status": clamp_status,
        "door_status": door_state,
        "current_door_cmd": current_door_cmd,
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
    result = client.publish(tracking_topic, msg)
    if result[0] == 0:
        print(f"Sent tracking data to topic `{tracking_topic}`")
    else:
        print(f"Failed to send tracking data")

# ??????
def LCS_event(client):
    global event_status, event_level
    current_time = time.time()
    data = {
        "supplier_sn": supplier_sn,
        "timestamp": current_time,
        "event": [
            {
                "event_name": "Stuck",
                "event_level": event_level,
                "event_status": event_status,
            }
        ]
    }
    msg = json.dumps(data)
    result = client.publish(event_topic, msg)
    if result[0] == 0:
        print(f"Sent event data to topic `{event_topic}`")
    else:
        print(f"Failed to send event data")

# ???????
def LCS_LDR_ans(client, message_id, robot_name):
    current_time = time.time()
    data = {
        "message_id": message_id,
        "timestamp": current_time,
        "robot_name": robot_name
    }
    msg = json.dumps(data)
    result = client.publish(lcs_to_robot_ack_topic, msg)
    if result[0] == 0:
        print(f"Sent response to topic `{lcs_to_robot_ack_topic}`")
    else:
        print(f"Failed to send response")

# ??FMS??
def LCS_fms_ans(client, message_id, device_name):
    current_time = time.time()
    data = {
        "message_id": message_id,
        "timestamp": current_time,
        "device_name": device_name
    }
    msg = json.dumps(data)
    result = client.publish(lcs_to_fms_ack_topic, msg)
    if result[0] == 0:
        print(f"Sent response to topic `{lcs_to_fms_ack_topic}`")
    else:
        print(f"Failed to send response")

################# ????? ##############################

# ?????
def execute_door_operation(door_cmd, solenoid_pub, message_id, device_name):
    global door_state, current_door_cmd, door_operation_start, door_operation_running
    global last_door_state
    
    # ??????
    current_door_cmd = door_cmd
    door_operation_start = time.time()
    door_operation_running = True
    last_door_state = door_state  # ?????
    
    # ?????????
    solenoid_msg = SolenoidInfo()
    if door_cmd == "open_door":
        solenoid_msg.solenoid_ctr = dooropen_bool
        print(f"??????,??{door_operation_time}?")
    elif door_cmd == "close_door":
        solenoid_msg.solenoid_ctr = doorclose_bool
        print(f"??????,??{door_operation_time}?")
    solenoid_pub.publish(solenoid_msg)
    
    # ??????????
    elapsed_time = 0
    while elapsed_time < door_operation_time and door_operation_running:
        time.sleep(0.1)  # ?????,?????
        elapsed_time = time.time() - door_operation_start
        
        # ???????????????
        if current_door_cmd != door_cmd and current_door_cmd is not None:
            print(f"?????,???: {current_door_cmd}")
            break
    
    # ??????
    if current_door_cmd == door_cmd:
        # ??????
        if door_cmd == "open_door":
            door_state = 1
            print("??????")
        elif door_cmd == "close_door":
            door_state = 0
            print("??????")
    else:
        # ????????
        print(f"???{door_cmd}???")
    
    # ??????
    door_operation_running = False
    current_door_cmd = None
    
    # ????????MQTT
    if last_door_state != door_state:
        rospy.loginfo(f"Door state updated: {last_door_state} -> {door_state}")


################# MQTT???? ##############################

def on_message(client, userdata, msg):
    global door_operation_running, current_door_cmd, door_cmd
    
    try:
        if msg.topic == robot_to_lcs_topic:
            # ???????
            payload = msg.payload.decode("utf-8")
            print(f"Received message: {payload} from topic: {msg.topic}")
            data = json.loads(payload)
            message_id = data.get("message_id", 0)
            robot_name = data.get("robot_name")
            clamp_cmd = data.get("action")
            
            LCS_LDR_ans(client, message_id, robot_name)
            client.publish(con_topic, "clamp cmd requested")

            if clamp_cmd == "lcs_clamp":
                solenoid_msg = SolenoidInfo()
                solenoid_msg.solenoid_ctr = clamp_bool
                ros_pub_clamp.publish(solenoid_msg)
                time.sleep(8)

            elif clamp_cmd == "lcs_unclamp":
                solenoid_msg = SolenoidInfo()
                solenoid_msg.solenoid_ctr = unclamp_bool
                ros_pub_clamp.publish(solenoid_msg)
                time.sleep(8)

                
        elif msg.topic == fms_to_robot_topic:
            # ??FMS???
            payload = msg.payload.decode("utf-8")
            print(f"Received message: {payload} from topic: {msg.topic}")
            data = json.loads(payload)
            message_id = data.get("message_id", 0)
            device_name = data.get("device_name")
            door_cmd = data.get("action")
            
            LCS_fms_ans(client, message_id, device_name)
            client.publish(con_topic, f"door cmd requested: {door_cmd}")
            
            # ????????????
            if door_operation_running and current_door_cmd != door_cmd:
                print(f"??????,???: {door_cmd}")
            
            # ?????
            if door_cmd == "open_door":
                # ??????
                open_thread = threading.Thread(
                    target=execute_door_operation,
                    args=("open_door", ros_pub_clamp, message_id, device_name)
                )
                open_thread.daemon = True
                open_thread.start()
                
            elif door_cmd == "close_door":
                # ??????
                close_thread = threading.Thread(
                    target=execute_door_operation,
                    args=("close_door", ros_pub_clamp, message_id, device_name)
                )
                close_thread.daemon = True
                close_thread.start()
                
            else:
                # ????,??????
                solenoid_msg = SolenoidInfo()
                solenoid_msg.solenoid_ctr = waitting_bool
                ros_pub_clamp.publish(solenoid_msg)
                client.publish(con_topic, "no valid door cmd requested")
                
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
    except Exception as e:
        print(f"An error occurred in message processing: {e}")

################# ??? ##############################

# ????
def run():
    global client  # ???????
    client = connect_mqtt()
    client.on_message = on_message
    client.loop_start()

    try:
        while not rospy.is_shutdown():
            LCS_tracking(client)
            time.sleep(0.2)  # ????????,?????
    except KeyboardInterrupt:
        print("Exiting program")
    finally:
        client.loop_stop()
        client.disconnect()

# ????
if __name__ == '__main__':
    rospy.init_node('mqtt_ros_bridge', anonymous=True)

    # ROS???
    rospy.Subscriber("/lcs/full_state", Bool, full_state_callback)
    rospy.Subscriber("/lcs/full_precent", Float32, full_state_high_callback)
    rospy.Subscriber("/lcs/state/ls", LimitSwitchInfo, limit_switch_callback)
    rospy.Subscriber("/lcs/ctr/solenoid", SolenoidInfo, solenoid_callback)
    rospy.Subscriber("/lcs/stuck_state", Bool, stuck_state_callback)

    # ROS???
    ros_pub_clamp = rospy.Publisher("/lcs/ctr/solenoid", SolenoidInfo, queue_size=10)

    run()
