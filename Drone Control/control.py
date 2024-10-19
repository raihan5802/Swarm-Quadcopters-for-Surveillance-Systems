from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

# Connect to the vehicle
print("Connecting to vehicle on: /dev/ttyACM0")
vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

def check_gps():
    """
    Check if GPS fix is available and stable.
    """
    print("Checking GPS fix")
    while vehicle.gps_0.fix_type < 3:  # Ensure at least a 3D fix
        print(" Waiting for GPS fix...")
        time.sleep(1)
    
    initial_location = vehicle.location.global_frame
    print("Initial GPS location acquired")
    time.sleep(2)  # wait to check GPS stability
    
    second_location = vehicle.location.global_frame
    distance_moved = get_distance_metres(initial_location, second_location)
    
    if distance_moved > 0.5:  # threshold for GPS stability
        print("GPS instability detected")
        return False
    print("GPS is stable")
    return True

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlon = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5

def arm_and_takeoff(target_altitude):
    """
    Arms vehicle and fly to target altitude with gradual throttle increase.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    
    if not check_gps():
        print("GPS check failed. Aborting takeoff.")
        return

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    thrust = 0.55
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        elif current_altitude < target_altitude * 0.9:
            thrust += 0.05
        else:
            thrust -= 0.05
        vehicle.channels.overrides['3'] = thrust
        print(" Altitude: ", current_altitude)
        time.sleep(1)
    
    vehicle.channels.overrides['3'] = None  # clear throttle override

# Arm and take off to 1 meter
arm_and_takeoff(1)

# Switch to LOITER mode to maintain position
print("Switching to LOITER mode")
vehicle.mode = VehicleMode("LOITER")

# Hold position for 10 seconds
print("Holding position in LOITER mode for 10 seconds")
time.sleep(10)

# Land the vehicle
print("Landing")
vehicle.mode = VehicleMode("LAND")

# Wait until the vehicle lands
while vehicle.armed:
    print(" Waiting for landing...")
    time.sleep(1)

# Disarm the vehicle
print("Disarming")
vehicle.armed = False

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

print("Completed")
