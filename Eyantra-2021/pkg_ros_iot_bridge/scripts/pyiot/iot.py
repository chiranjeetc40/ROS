"""
This script connecct Mqtt topic to Ros topic.
It update data to google sheet.
"""
from multiprocessing.dummy import Pool
import time
import requests
import paho.mqtt.client as mqtt #import the client1


# -----------------  MQTT SUB -------------------

def on_connect(client, userdata, flags, rc):
    """
    This function is executed when connection to Mqtt topic is established.
    """
    print("[INFO] Connected to MQTT topic With Result Code: " + str(rc))

    
def mqtt_subscribe_thread_start(arg_callback_func,arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_qos):
    """
    This function is called to subscribe to particular Mqtt topic.
    This function run in different thread and publish data to ROS topic when new message is recieved on topic.
    Parameters:
        arg_callback_func: This is callback function called to publish message to Ros topic.
        arg_broker_url: Address of server which act as Broker.
        arg_broker_port: Port no connect to broker.
        arg_mqtt_topic: Mqtt topic on which message is published.
        arg_mqtt_qos: QoS no which defines quality of message is sent.
    Returns:
        0 if sucess and -1 if failed.
    """
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1) # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()    # starts a new thread
        return 0
    except Exception as e:
    	print(str(e))
        return -1
        

# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos):
    """
    This function is called to publish Message to particular Mqtt topic in new thread.
    Parameters:
        arg_broker_url: Address of server which act as Broker.
        arg_broker_port: Port no connect to broker.
        arg_mqtt_topic: Mqtt topic on which message is published.
        arg_mqtt_message: Message to be published on Mqtt topic.
        arg_mqtt_qos: QoS no which defines quality of message is sent.
    Returns:
        0 if sucess and -1 if failed.
    """
    try:        
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.5) # wait

        #mqtt_client.loop_stop() #stop the loop
        return 0
    except Exception as e:
    	print(str(e))
        return -1
        
# -----------------  Google Sheet PUSH -------------------
def sheet_push(webapp_id,parameters):
    """
    This function is called update data to google sheet.
    Parameters:
        webapp_id: Unique webapp id of sheet.
        parameters: Content to be updated on sheet.
    Returns:
        0 if sucess and -1 if failed.
    """
    print("Sending data to Sheet")	
    try:
        URL = "https://script.google.com/macros/s/"+webapp_id+"/exec"
        response = requests.get(URL, params=parameters)
        if response.content=="success":
            return 0
        else:
            print("Trying again to upload data")
            sheet_push(webapp_id,parameters)
    except Exception as e:
        print(str(e))
        sheet_push(webapp_id,parameters)
        return -1

        
	
