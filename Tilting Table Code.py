import time
import board
import digitalio
from adafruit_motor import stepper
from adafruit_motor import servo
import analogio
import pwmio

# Set DELAY as the length of time that the motor runs for each step. Adafruit suggests 0.01, but ME 30 folks have found this delay value needs to be much shorter. If 0.01 doesn't work, try something as short as 0.005.

delay = 0.003

# Set the number of steps for one rotation of the motor. For the stepper motor in the ME 30 kit, one rotation takes 200 steps.

steps = 200

maxServoAngle = 10  # Maximum angle for servo motor
servoOffset = 88

maxStepperAngle = 10
num_microsteps = 10
maxSteps = int(maxStepperAngle/(1.8/num_microsteps)) # Max steps from center
stepPosition = 0  # Current step position
stepError = 0  # Number of steps between current position and desired position dictated by joystick

joystickRange = 61000/2  # Maximum joystick analog value
midJoystick = 34000

# You can use any available GPIO pin on a microcontroller.
# The following pins are simply a suggestion. If you use different pins,
# update the following code to use your chosen pins.

coils = (
    pwmio.PWMOut(board.D4, frequency=1500),  # A1
    pwmio.PWMOut(board.D5, frequency=1500),  # A2
    pwmio.PWMOut(board.D6, frequency=1500),  # A3
    pwmio.PWMOut(board.D7, frequency=1500),  # A4
)

xJoystick = analogio.AnalogIn(board.A1)
yJoystick = analogio.AnalogIn(board.A2)
buttonJoystick = digitalio.DigitalInOut(board.D3)
pwm = pwmio.PWMOut(board.D2, frequency=50)

yAxisServo = servo.Servo(pwm, min_pulse=750, max_pulse=2250)

"""
for coil in coils:
    coil.direction = digitalio.Direction.OUTPUT

"""

# The next line is essential. It creates an
# object, 'motor' (you can name it anything)
# to hold the details about your stepper motor wiring.
# It uses the stepper command from the adafruit_motor library.

motor = stepper.StepperMotor(coils[0], coils[1], coils[2], coils[3], microsteps=num_microsteps)

# The for loop below represents just one approach to taking a step.
# For more info on the arguments for the 'onestep' command, see
# the "Stepper Motors" section of this page:
# https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/circuitpython


while True:

    stepError = int(((xJoystick.value-midJoystick)/joystickRange)*maxSteps - stepPosition)
    if abs(stepError) > 1:
        print("step error:", stepError, "current position:", stepPosition, "joystick:", xJoystick.value)
        if stepError > 0:
            motor.onestep(direction=stepper.FORWARD, style=stepper.MICROSTEP)
            time.sleep(delay)
            stepPosition += 1
        else:
            motor.onestep(direction=stepper.BACKWARD, style=stepper.MICROSTEP)
            time.sleep(delay)
            stepPosition -= 1
    yAxisAngle = (-(yJoystick.value-midJoystick)/joystickRange)*maxServoAngle+servoOffset
    yAxisServo.angle = yAxisAngle

