import time
from pymavlink import mavutil

# Connect to the Pixhawk via USB
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# Wait for the first heartbeat
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Heartbeat received!")

def get_gps_data():
    while True:
        msg = connection.recv_match(blocking=True)
        if not msg:
            continue

        # Check for GPS_RAW_INT message
        if msg.get_type() == 'GPS_RAW_INT':
            lat = msg.lat / 1e7  # Latitude in degrees
            lon = msg.lon / 1e7  # Longitude in degrees
            alt = msg.alt / 1000  # Altitude in meters
            eph = msg.eph / 100  # GPS HDOP
            epv = msg.epv / 100  # GPS VDOP
            fix_type = msg.fix_type  # GPS fix type
            satellites_visible = msg.satellites_visible  # Number of satellites visible

            print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt} meters, HDOP: {eph}, VDOP: {epv}, Fix type: {fix_type}, Satellites visible: {satellites_visible}")

        # Check for GPS_STATUS message
        elif msg.get_type() == 'GPS_STATUS':
            satellites_visible = msg.satellites_visible
            print(f"Satellites visible: {satellites_visible}")

        time.sleep(1)  

if __name__ == "__main__":
    try:
        get_gps_data()
    except KeyboardInterrupt:
        print("Program interrupted by user. Exiting...")
