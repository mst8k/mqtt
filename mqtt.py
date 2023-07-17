import serial
from time import sleep
import os,sys
from enum import Enum
import RPi.GPIO as GPIO
import paho.mqtt.client as paho
import urllib.parse as urlparse

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
LED_PIN=11  #define LED pin
GPIO.setup(LED_PIN,GPIO.OUT)   # Set pin function as output

selected_room = "A"
mute = 0

# Configure UART parameters
uart_device = '/dev/ttyS0'  # UART device path (UART0 on Raspberry Pi 4)
baudrate = 4800
ser = serial.Serial (uart_device, baudrate, timeout=1)    #Open port with baud rate

# Function to set the selected room
def set_selected_room(room):
    global selected_room
    selected_room = room

def set_mute(m):
    global mute
    mute = m

def write_to_device(data):
    ser.write(data)
    ser.flush()

class Command(Enum):
    NO_COMMAND = 0
    ROOM_A_POWER_ON = 1
    ROOM_A_POWER_OFF = 2
    ROOM_A_SOURCE_FM = 3
    ROOM_A_SOURCE_CD = 4
    ROOM_A_SOURCE_AUX = 5
    ROOM_A_SOURCE_VIDEO1 = 6
    ROOM_A_SOURCE_VIDEO2 = 7
    ROOM_A_SOURCE_TAPE = 8
    ROOM_A_VOL_UP = 9
    ROOM_A_VOL_DOWN = 10
    ROOM_A_MUTE = 11
    ROOM_A_UNMUTE = 12
    ROOM_B_POWER_ON = 13
    ROOM_B_POWER_OFF = 14
    ROOM_B_SOURCE_FM = 15
    ROOM_B_SOURCE_CD = 16
    ROOM_B_SOURCE_AUX = 17
    ROOM_B_SOURCE_VIDEO1 = 18
    ROOM_B_SOURCE_VIDEO2 = 19
    ROOM_B_SOURCE_TAPE = 20
    ROOM_B_VOL_UP = 21
    ROOM_B_VOL_DOWN = 22
    ROOM_B_MUTE = 23
    ROOM_B_UNMUTE = 24
    NUMBER_OF_COMMANDS = 25

# Dictionary with Command enum keys and corresponding values
command_str_collection = {
    Command.NO_COMMAND: " ",
    Command.ROOM_A_POWER_ON: "MM 4a,41,01",
    Command.ROOM_A_POWER_OFF: "MM 4a,43,01",
    Command.ROOM_A_SOURCE_FM: "MM 42,40,01,01",
    Command.ROOM_A_SOURCE_CD: "MM 42,40,01,02",
    Command.ROOM_A_SOURCE_AUX: "MM 42,40,01,03",
    Command.ROOM_A_SOURCE_VIDEO1: "MM 42,40,01,04",
    Command.ROOM_A_SOURCE_VIDEO2: "MM 42,40,01,05",
    Command.ROOM_A_SOURCE_TAPE: "MM 42,40,01,06",
    Command.ROOM_A_VOL_UP: "MM 44,41,00",
    Command.ROOM_A_VOL_DOWN: "MM 44,42,00",
    Command.ROOM_A_MUTE: "MM 43,41,01",
    Command.ROOM_A_UNMUTE: "MM 43,40,01",
    Command.ROOM_B_POWER_ON: "MM 4a,41,02",
    Command.ROOM_B_POWER_OFF: "MM 4a,43,02",
    Command.ROOM_B_SOURCE_FM: "MM 42,40,02,01",
    Command.ROOM_B_SOURCE_CD: "MM 42,40,02,02",
    Command.ROOM_B_SOURCE_AUX: "MM 42,40,02,03",
    Command.ROOM_B_SOURCE_VIDEO1: "MM 42,40,02,04",
    Command.ROOM_B_SOURCE_VIDEO2: "MM 42,40,02,05",
    Command.ROOM_B_SOURCE_TAPE: "MM 42,40,02,06",
    Command.ROOM_B_VOL_UP: "MM 44,41,01",
    Command.ROOM_B_VOL_DOWN: "MM 44,42,01",
    Command.ROOM_B_MUTE: "MM 43,41,02",
    Command.ROOM_B_UNMUTE: "MM 43,40,02",
    Command.NUMBER_OF_COMMANDS: " ",
}

def handle_command(command_enum):
    if not isinstance(command_enum, Command):
        raise ValueError("Invalid command_enum! Please pass a valid Command enum.")

    command_string = command_str_collection.get(command_enum, "Invalid command!")
    write_to_device(command_string)
    print(command_string)


def on_connect(self, mosq, obj, rc):
        self.subscribe("power", 0)
        self.subscribe("source", 0)
        self.subscribe("zone", 0)
        self.subscribe("volume", 0)

def handle_power_message(msg):
    power = msg.payload.decode()
    if selected_room == "A":
        if power == "on":
            handle_command(Command.ROOM_A_POWER_ON)
            print(f"ROOM A Received power: {msg.payload.decode()}")
        elif power == "off":
            handle_command(Command.ROOM_A_POWER_OFF)
            print(f"ROOM A Received power: {msg.payload.decode()}")
    else:
        if power == "on":
            handle_command(Command.ROOM_B_POWER_ON)
            print(f"ROOM B Received power: {msg.payload.decode()}")
        elif power == "off":
            handle_command(Command.ROOM_B_POWER_OFF)
            print(f"ROOM B Received power: {msg.payload.decode()}")

def handle_source_message(msg):
    source = msg.payload.decode()
    if selected_room == "A":
        if source == "tv":
            handle_command(Command.ROOM_A_SOURCE_AUX)
            print(f"ROOM A Received power: {msg.payload.decode()}")
        elif source == "music":
            handle_command(Command.ROOM_A_SOURCE_TAPE)
            print(f"ROOM A Received power: {msg.payload.decode()}")
    else:
        if source == "tv":
            handle_command(Command.ROOM_B_SOURCE_AUX)
            print(f"ROOM B Received power: {msg.payload.decode()}")
        elif source == "music":
            handle_command(Command.ROOM_B_SOURCE_TAPE)
            print(f"ROOM B Received power: {msg.payload.decode()}")

def handle_volume_message(msg):
    volume = msg.payload.decode()
    if selected_room == "A":   
        if volume == "up":
            handle_command(Command.ROOM_A_VOL_UP)
            print(f"ROOM A Received power: {msg.payload.decode()}")
        elif volume == "down":
            handle_command(Command.ROOM_A_VOL_DOWN)
            print(f"ROOM A Received power: {msg.payload.decode()}")
        elif volume == "mute":
            if mute == 0:
                set_mute(1)
                handle_command(Command.ROOM_A_MUTE)
                print("ROOM A  Mute")
            else:
                set_mute(0)
                handle_command(Command.ROOM_A_UNMUTE)
                print("ROOM A UnMute")
    else:
        if volume == "up":
            handle_command(Command.ROOM_B_VOL_UP)
            print(f"Received power: {msg.payload.decode()}")
        elif volume == "down":
            handle_command(Command.ROOM_B_VOL_DOWN)
            print(f"Received power: {msg.payload.decode()}")
        elif volume == "mute":
            if mute == 0:
                set_mute(1)
                handle_command(Command.ROOM_B_MUTE)
                print("ROOM B Mute")
            else:
                set_mute(0)
                handle_command(Command.ROOM_B_UNMUTE)
                print("ROOM B UnMute")
            

def handle_zone_message(msg):
    zone = msg.payload.decode()
    if zone == "salon":
        set_selected_room("A")
        print(f"Received power: {msg.payload.decode()}")
    elif zone == "kitchen":
        set_selected_room("B")
        print(f"Received power: {msg.payload.decode()}")

def on_message(client, userdata, msg):
    ser.write(f"Received message on topic '{msg.topic}': {msg.payload.decode()}")
    print(f"Received message on topic '{msg.topic}': {msg.payload.decode()}")
    if msg.topic == "power":
        handle_power_message(msg)
    elif msg.topic == "volume":
        handle_volume_message(msg)
    elif msg.topic == "source":
        handle_source_message(msg)
    elif msg.topic == "zone":
        handle_zone_message(msg)

def on_publish(mosq, obj, mid):
    print("mid: " + str(mid))

    
def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))



mqttc = paho.Client()                        # object declaration
# Assign event callbacks
mqttc.on_message = on_message                          # called as callback
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe


#url_str = os.environ.get('CLOUDMQTT_URL', 'tcp://broker.emqx.io:1883')                  # pass broker addr e.g. "tcp://iot.eclipse.org"
#url_str = os.environ.get('CLOUDMQTT_URL', 'tcp://broker.hivemq.com:1883')
url_str = os.environ.get('CLOUDMQTT_URL', 'tcp://broker.emqx.io:1883') 
url = urlparse.urlparse(url_str)
mqttc.connect(url.hostname, url.port)

rc = 0
while True:
    while rc == 0:
        import time   
        rc = mqttc.loop()
        #time.sleep(0.5)
    print("rc: " + str(rc))
