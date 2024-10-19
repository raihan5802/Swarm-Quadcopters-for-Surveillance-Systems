import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set pins
TRIG = 23  # pin 23 to TRIG
ECHO = 24  # pin 24 to ECHO

# Set up TRIG and ECHO pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    # Set TRIG LOW
    GPIO.output(TRIG, False)
    time.sleep(2)

    # Send 10us pulse to TRIG
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Initialize pulse_start and pulse_end
    pulse_start = 0
    pulse_end = 0

    print("Waiting for ECHO to go HIGH...")
    start_time = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start - start_time > 1:
            print("Timeout: ECHO did not go HIGH")
            return -1

    print("ECHO is HIGH, timing the pulse...")
    start_time = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end - start_time > 1:
            print("Timeout: ECHO did not go LOW")
            return -1

    print("Pulse received. Calculating distance...")
    # Calculate the difference in times
    pulse_duration = pulse_end - pulse_start

    # Multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    print("Distance calculated: {:.2f} cm".format(distance))
    return distance

if __name__ == '__main__':
    try:
        while True:
            print("Measuring distance...")
            dist = get_distance()
            if dist != -1:
                print("Measured Distance = {:.2f} cm".format(dist))
            else:
                print("Measurement failed")
            time.sleep(1)

    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
