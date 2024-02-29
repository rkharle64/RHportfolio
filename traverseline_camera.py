import RPi.GPIO as GPIO
import time
import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import controls

# Set pin values for right motor
ena = 7
in1 = 11
in2 = 13

# Set pin values for left motor
enb = 12
in3 = 16
in4 = 18

# Board and pin setup
GPIO.setwarnings(False)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(ena, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)

GPIO.setup(enb, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)

# Set all pins low to start to prevent rotation on run
GPIO.output(ena, GPIO.LOW)
GPIO.output(in1, GPIO.LOW) #Change which pin (in1 or in2) is set to high and which is set to low to switch direction
GPIO.output(in2, GPIO.HIGH)

GPIO.output(enb, GPIO.LOW)
GPIO.output(in3, GPIO.HIGH)
GPIO.output(in4, GPIO.LOW)

# Create PWM instance on enable pin A at 50 Hz. Setting a really low duy cycle (5 Hz) allows you to pulse the motor slowly
motorR = GPIO.PWM(ena, 50)
motorL = GPIO.PWM(enb, 50)

motorR.start(0)
motorL.start(0)

# Initialize camera
picam2 = Picamera2() # assigns camera variable
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # sets auto focus mode
picam2.start() # activates camera

time.sleep(1) # wait to give camera time to start up


def set_left_motor_speed(motor, speed, in_pin1, in_pin2):
    if 0 <= speed <= 10:
        motor.ChangeDutyCycle(0)
        GPIO.output(in_pin1, GPIO.LOW)
        GPIO.output(in_pin2, GPIO.LOW)
    elif speed > 10:
        motor.ChangeDutyCycle(speed)
        GPIO.output(in_pin1, GPIO.HIGH)
        GPIO.output(in_pin2, GPIO.LOW)
    else:  # speed < -10
        motor.ChangeDutyCycle(-speed)  # Use the absolute value of the negative speed
        GPIO.output(in_pin1, GPIO.LOW)
        GPIO.output(in_pin2, GPIO.HIGH)

def set_right_motor_speed(motor, speed, in_pin1, in_pin2):
    if 0 <= speed <= 10:
        motor.ChangeDutyCycle(0)
        GPIO.output(in_pin1, GPIO.LOW)
        GPIO.output(in_pin2, GPIO.LOW)
    elif speed > 10:
        motor.ChangeDutyCycle(speed)
        GPIO.output(in_pin1, GPIO.LOW)
        GPIO.output(in_pin2, GPIO.HIGH)
    else:  # speed < -10
        motor.ChangeDutyCycle(-speed)  # Use the absolute value of the negative speed
        GPIO.output(in_pin1, GPIO.HIGH)
        GPIO.output(in_pin2, GPIO.LOW)

try:
    setpoint = 289
    Kp = .028
    Kd = 0.00
    
    L_old_speed = 16
    R_old_speed = 16
    p_error = 0

    while True:
        
    # Calculate centroid values
        image = picam2.capture_array("main")
        cv2.imshow('img',image)
    
        # Crop the image
        crop_img = image[0:2500, 0:4600]
    
        # Convert to grayscale
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    
        # Gaussian blur
        blur = cv2.GaussianBlur(gray,(5,5),0)
    
        # Color thresholding
        input_threshold,comp_threshold = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)
    
        # Find the contours of the frame
        contours,hierarchy = cv2.findContours(comp_threshold.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
        # Find the biggest contour (if detected)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c) # determine moment - weighted average of intensities

            if int(M['m00']) != 0:
                cx = int(M['m10']/M['m00']) # find x component of centroid location
                cy = int(M['m01']/M['m00']) # find y component of centroid location
            else:
                print("Centroid calculation error, looping to acquire new values")
                continue
            cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1) # display vertical line at x value of centroid
            cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1) # display horizontal line at y value of centroid
    
            cv2.drawContours(crop_img, contours, -1, (0,255,0), 2) # display green lines for all contours
            
            # determine location of centroid in x direction and adjust steering recommendation
            #print("CX:" + str(cx))
            #print("CY:" + str(cy))
    
        # Display the resulting frame
        cv2.imshow('frame',crop_img)
        
        # Show image for 1 ms then continue to next image
        cv2.waitKey(1)
    # PID control
        c_error = setpoint - cx

        prop = c_error*Kp
        deriv = Kd*(c_error - p_error)
        
        change = prop + deriv

        L_new_speed = L_old_speed - change
        R_new_speed = R_old_speed + change

        set_left_motor_speed(motorL, L_new_speed, in3, in4)
        set_right_motor_speed(motorR, R_new_speed, in1, in2)
    '''
    # Check that speed isn't too low
        if L_new_speed < 10:
            L_new_speed = 10
       
        if R_new_speed < 10:
            R_new_speed = 10
        
        print(L_new_speed)

    # Set motor speeds
        motorL.ChangeDutyCycle(L_new_speed)
        motorR.ChangeDutyCycle(R_new_speed)
        #L_old_speed = L_new_speed
        #R_old_speed = R_new_speed
        
        p_error = c_error
    '''     
except KeyboardInterrupt:
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    print("Exiting the script.")
    GPIO.cleanup()