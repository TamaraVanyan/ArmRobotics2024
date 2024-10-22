import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Initialize the camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap.set(3, 160)  # Set width
cap.set(4, 120)  # Set height

# GPIO Pin Setup
in1 = 4
in2 = 17
in3 = 14
in4 = 15
en1 = 18
en2 = 19
servo_pin = 1   # Define the GPIO pin for the servo motor
vex_motor_pin = 26  # Define the GPIO pin for the VEX motor (Motor Controller 29)

# GPIO Mode setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(en1, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(vex_motor_pin, GPIO.OUT)

# PWM Setup for the motors and servo
p1 = GPIO.PWM(en1, 200)  # Left motor
p2 = GPIO.PWM(en2, 200)  # Right motor
servo = GPIO.PWM(servo_pin, 50)  # Servo motor at 50Hz
vex_motor = GPIO.PWM(vex_motor_pin, 50)  # VEX motor at 50Hz

# Start motors with an initial speed
initial_speed = 100
p1.start(initial_speed)
p2.start(initial_speed)
servo.start(0)  # Initially, the servo is inactive
vex_motor.start(0)  # Initially, the VEX motor is inactive

# Stop the motors initially
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)

# Ultrasonic sensor pins
TRIG1 = 23
ECHO1 = 24
TRIG2 = 27
ECHO2 = 22

# Setup for ultrasonic sensors
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.setup(TRIG2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)

# Function to calculate distance from an ultrasonic sensor
def distance(trigger, echo):
    GPIO.output(trigger, True)
    time.sleep(0.00001)  # Trigger pulse
    GPIO.output(trigger, False)

    start_time = time.time()
    stop_time = time.time()

    # Wait for echo to start
    while GPIO.input(echo) == 0:
        start_time = time.time()

    # Wait for echo to stop
    while GPIO.input(echo) == 1:
        stop_time = time.time()

    # Calculate time difference and distance
    time_elapsed = stop_time - start_time
    dist = (time_elapsed * 34300) / 2  # Distance in cm
    return dist

# Function to control the VEX motor movement (open/close hand)
def move_vex_motor(duty_cycle, duration):
    vex_motor.ChangeDutyCycle(duty_cycle)
    time.sleep(duration)
    vex_motor.ChangeDutyCycle(7.5)  # Neutral position to stop movement

# Function to control the servo motor movement
def move_servo(duty_cycle, duration):
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(duration)
    servo.ChangeDutyCycle(0)  # Stop the servo after the movement

# Function to handle bottle operation (open, close, servo down/up)
def handle_bottle_operation():
    # Open hand with VEX motor
    print("Opening hand with VEX motor")
    move_vex_motor(10, 1)  # Open hand (10% duty cycle)

    # Move servo down
    print("Moving servo down")
    move_servo(7, 1)  # Move servo down (7% duty cycle)

    # Close hand with VEX motor
    print("Closing hand with VEX motor")
    move_vex_motor(3, 1)  # Close hand (3% duty cycle)

    # Move servo up
    print("Moving servo up")
    move_servo(3, 1)  # Move servo up (3% duty cycle)

    time.sleep(2)

    print("Moving servo down")
    move_servo(7, 1)  # Move servo down (7% duty cycle)

    print("Opening hand with VEX motor")
    move_vex_motor(10, 1)  # Open hand (10% duty cycle)

    print("Moving servo up")
    move_servo(3, 1)  # Move servo up (3% duty cycle)

    print("Closing hand with VEX motor")
    move_vex_motor(3, 1)  # Close hand (3% duty cycle)

    # Stop both motors and return control to the robot's movement
    print("Stopping VEX and servo motors. Robot will continue movement.")
    vex_motor.ChangeDutyCycle(0)  # Stop VEX motor
    servo.ChangeDutyCycle(0)  # Stop servo motor

# Main control loop
try:
    a = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV range for detecting black, blue, and green colors
        low_b = np.array([0, 0, 0])
        high_b = np.array([180, 255, 30])
        low_blue = np.array([90, 50, 50])
        high_blue = np.array([130, 255, 255])
        low_green = np.array([40, 50, 50])
        high_green = np.array([80, 255, 255])

        # Create binary masks for the black, blue, and green colors
        mask_black = cv2.inRange(hsv, low_b, high_b)
        mask_blue = cv2.inRange(hsv, low_blue, high_blue)
        mask_green = cv2.inRange(hsv, low_green, high_green)

        # Find contours for black, blue, and green colors
        contours_black, _ = cv2.findContours(mask_black, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Check if blue color is detected (to trigger bottle operation)
        if any(cv2.contourArea(c) > 500 for c in contours_blue):
            if a == 0:
                a = 1
                print("Blue detected, executing bottle operation...")
                handle_bottle_operation()

        # Check if green color is detected (restore normal speed)
        elif any(cv2.contourArea(c) > 600 for c in contours_green):
            print("Green detected, returning to normal speed...")
            a = 0

        # Follow the black line
        elif contours_black:
            c = max(contours_black, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                print("CX : " + str(cx) + " CY : " + str(cy))

                if cx >= 100:
                    print("Turn Left")
                    p1.ChangeDutyCycle(100)
                    p2.ChangeDutyCycle(100)
                    GPIO.output(in1, GPIO.LOW)
                    GPIO.output(in2, GPIO.HIGH)
                    GPIO.output(in3, GPIO.LOW)
                    GPIO.output(in4, GPIO.HIGH)
                elif 65 < cx < 100:
                    print("On Track!")
                    p1.ChangeDutyCycle(100)
                    p2.ChangeDutyCycle(100)
                    GPIO.output(in1, GPIO.LOW)
                    GPIO.output(in2, GPIO.HIGH)
                    GPIO.output(in3, GPIO.HIGH)
                    GPIO.output(in4, GPIO.LOW)
                elif cx < 65:
                    print("Turn Right")
                    p1.ChangeDutyCycle(100)
                    p2.ChangeDutyCycle(100)
                    GPIO.output(in1, GPIO.HIGH)
                    GPIO.output(in2, GPIO.LOW)
                    GPIO.output(in3, GPIO.HIGH)
                    GPIO.output(in4, GPIO.LOW)

                # Draw a circle at the centroid
                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
        else:
            print("I don't see the line")
            dist1 = distance(TRIG1, ECHO1)
            dist2 = distance(TRIG2, ECHO2)

            print(f"Distance from Sensor 1: {dist1:.2f} cm")
            print(f"Distance from Sensor 2: {dist2:.2f} cm")
            
            
            if dist1 - dist2 > 5:
                while dist1 - dist2 > 5:
                    dist1 = distance(TRIG1, ECHO1)
                    dist2 = distance(TRIG2, ECHO2)
                    print("Turn Left")
                    p1.ChangeDutyCycle(70)
                    p2.ChangeDutyCycle(70)
                    GPIO.output(in1, GPIO.LOW)
                    GPIO.output(in2, GPIO.HIGH)
                    GPIO.output(in3, GPIO.LOW)
                    GPIO.output(in4, GPIO.HIGH)
            elif dist2 - dist1 > 5:
                while dist2 - dist1 > 5:
                    dist1 = distance(TRIG1, ECHO1)
                    dist2 = distance(TRIG2, ECHO2)
                    print("Turn Right")
                    p1.ChangeDutyCycle(70)
                    p2.ChangeDutyCycle(70)
                    GPIO.output(in1, GPIO.HIGH)
                    GPIO.output(in2, GPIO.LOW)
                    GPIO.output(in3, GPIO.HIGH)
                    GPIO.output(in4, GPIO.LOW)
            else:
                print("Move Straight")
                p1.ChangeDutyCycle(90)
                p2.ChangeDutyCycle(90)
                GPIO.output(in1, GPIO.LOW)
                GPIO.output(in2, GPIO.HIGH)
                GPIO.output(in3, GPIO.HIGH)
                GPIO.output(in4, GPIO.LOW)

        # Show the frame for debugging
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()

