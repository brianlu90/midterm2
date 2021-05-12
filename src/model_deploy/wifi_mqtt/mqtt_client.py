import paho.mqtt.client as paho
import serial
import time
import matplotlib.pyplot as plt
import numpy as np

# https://os.mbed.com/teams/mqtt/wiki/Using-MQTT#python-client

# MQTT broker hosted on local machine
mqttc = paho.Client()
serdev = '/dev/ttyACM0'
s = serial.Serial(serdev, 9600)

# Settings for connection
# TODO: revise host to your IP
host = "192.168.43.162"
topic1= "Cls Gesture"
topic2= "Ext Feature"
i = 0
j = 0
ges = []
fea = []

# Callbacks
def on_connect(self, mosq, obj, rc):
    print("Connected rc: " + str(rc))

def on_message(mosq, obj, msg):
    global i
    global j
    if msg.topic  == topic1:
        if (i < 5):
            print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n");
            i = i + 1
        else:
            j = 0;
            s.write("/GestureUI_End/run\r\n".encode())
            print("Start geting feature...")
            time.sleep(0.5)
            s.write("/Feature/run\r\n".encode())
    elif msg.topic  == topic2:
        if j < 5:
            print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n");
            j = j + 1
        else :
            i = 0;
            s.write("/GestureUI/run\r\n".encode())

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
    print("Unsubscribed OK")

# Set callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe

# Connect and subscribe
print("Connecting to " + host + "/" + topic1)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic1, 0)
print("Connecting to " + host + "/" + topic2)
mqttc.subscribe(topic2, 0)


s.write("/GestureUI/run\r\n".encode())
# Loop forever, receiving messages
mqttc.loop_forever()