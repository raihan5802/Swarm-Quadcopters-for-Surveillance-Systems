import paho.mqtt.client as mqtt
import time
import json
from geopy.distance import geodesic
from geopy.point import Point

# HiveMQ Cloud connection details
broker = '<placeholder>'  #placeholder for HiveMQ cluster id
port = 8883  # Standard port for secure MQTT connections
username = 'swarm1'  # HiveMQ Cloud username
password = 'swarm'  # HiveMQ Cloud password

client_id = 'pi1'

# MQTT topics
topic_send = 'drone1/coords'
topic_receive = 'drone2/coords'

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(topic_receive)

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    lat, lon = data['latitude'], data['longitude']
    print(f"Received from Pi2: Latitude={lat}, Longitude={lon}")
    new_coords = calculate_exact_distance(lat, lon, 2)
    print(f"New Coordinates: Latitude={new_coords.latitude}, Longitude={new_coords.longitude}")

def calculate_exact_distance(lat, lon, distance_meters):
    original_point = Point(lat, lon)
    new_point = geodesic(meters=distance_meters).destination(original_point, 0)
    actual_distance = geodesic((lat, lon), (new_point.latitude, new_point.longitude)).meters

    if abs(actual_distance - distance_meters) > 0.01:  # Allow small tolerance
        # Recalculate to ensure exact distance
        new_point = geodesic(meters=distance_meters).destination(original_point, 0)
    
    return new_point

client = mqtt.Client(client_id)
client.username_pw_set(username, password)
client.tls_set()  # Enable SSL/TLS
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker, port, 60)

client.loop_start()

try:
    while True:
        lat = float(input("Enter latitude for Pi1: "))
        lon = float(input("Enter longitude for Pi1: "))
        coords = {'latitude': lat, 'longitude': lon}
        client.publish(topic_send, json.dumps(coords))
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting")
    client.loop_stop()
    client.disconnect()
