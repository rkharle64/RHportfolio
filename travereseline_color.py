import RPi.GPIO as GPIO
import time

# # Pins for Motor Driver Inputs 
Motor1F = 33 #change when needed right motor
Motor1B = 31 #change when needed
Motor1S = 36 #change when needed

Motor2F = 35 #change when needed left motor
Motor2B = 37 #change when needed
Motor2S = 38 #change when needed


rgbmin = [27679.51112313652, 24363.108307484985, 21059.020229051708]
rgbmax = [51000.15807200788, 50949.35801659318, 51218.13141859301]


# Assign GPIO pin numbers to variables
s2 = 16
s3 = 18
sig = 22 #labeled "out" on your board
cycles = 1000

# Setup GPIO and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

DELAY = .01

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)


GPIO.setup(Motor1F,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor2F,GPIO.OUT)
GPIO.setup(Motor2B,GPIO.OUT)
GPIO.setup(Motor1S,GPIO.OUT)
GPIO.setup(Motor2S,GPIO.OUT)


leftEnPWM = GPIO.PWM(Motor2S, 6) #left motor
leftEnPWM.start(6)
GPIO.output(Motor2S, GPIO.HIGH)
 
rightEnPWM = GPIO.PWM(Motor1S, 5) #right motor
rightEnPWM.start(5)
GPIO.output(Motor1S, GPIO.HIGH)

def scanrval():
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.LOW)
    # time.sleep(0.1)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    red = cycles / duration
    return red

def scangval():
    GPIO.output(s2, GPIO.HIGH)
    GPIO.output(s3, GPIO.HIGH)
    start_time = time.time()
    for count in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    green = cycles / duration
    return green




def map(Fmin, Fmax, F):
    return 255 * ((F - Fmin)/(Fmax - Fmin))

def DetectColor(rgbmin, rgbmax, color):
    if ((color == "b") or (color == "g")):
        col = scanrval()
    else:
        col = scangval()
    maps = map(rgbmin[0],rgbmax[0] ,col)
    return maps


def obtainerror(expected, crgb):
    return (crgb - expected)

def newspeed(Cerr,kp):
    pidval = Cerr*kp
    return pidval

def move(speedl, speedr, change, ma):
    if change < 0:
        change = change * 5
    nspeedl = max(min(ma,speedl + change),0)
    nspeedr = max(min(ma,speedr - change),0)
    print("spinning fast")
    leftEnPWM.ChangeDutyCycle(nspeedl)
    rightEnPWM.ChangeDutyCycle(nspeedr)
    #time.sleep(DELAY)
    return nspeedl, nspeedr



def choose():
    color = input("What color you want?: ")
    if color == 'r':
        expected = 150 #expected green val
        kp = 1/700
        Cspdl = 2.5
        Cspdr = 2.5
        max = 5
    elif color == 'g':
        expected = 150 #expected red val
        kp = 1/700
        Cspdl = 2.5
        Cspdr = 2.5
        max = 5
    elif color == 'b':
        expected = 144 #expected red val
        kp = 1/300
        Cspdl = 5
        Cspdr = 5
        max = 10
    else:
        expected = 180 #expected green val
        kp = 1/700
        Cspdl = 2.5
        Cspdr = 2.5
        max = 5
    return expected, kp, Cspdl,  Cspdr, max, color


try:
    expected, kp, Cspdl, Cspdr, ma, color = choose()
    GPIO.output(Motor1F, GPIO.HIGH)
    GPIO.output(Motor2F, GPIO.HIGH)
    while True:
        col = DetectColor(rgbmin,rgbmax, color)
        Cerr = obtainerror(expected, col)
        print("error")
        print(Cerr)
        cisp = newspeed(Cerr,kp)
        print("speed")
        Cspdl,Cspdr = move(Cspdl,Cspdr,cisp, ma)
except KeyboardInterrupt:
    GPIO.output(Motor1F, GPIO.LOW)
    GPIO.output(Motor2F, GPIO.LOW)
    print("Exiting the script.")
    GPIO.cleanup()

##blue values,, scan red
    # expected = 100 #expected red val
    # kp = 1/300
    # Cspdl = 5
    # Cspdr = 5

## red values ,,, scan green
    # expected = 100 #expected red val
    # kp = 1/300
    # Cspdl = 5
    # Cspdr = 5   

##green values,,, scan red
    # expected = 180 #expected red val
    # kp = 1/700
    # Cspdl = 2.5
    # Cspdr = 2.5


## purple values scan green
    # expected = 100 #expected red val
    # kp = 1/300
    # Cspdl = 5
    # Cspdr = 5 