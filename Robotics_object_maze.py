from keras.models import load_model # TensorFlow is required for Keras to work
import cv2 # Install opencv-python
import numpy as np
from picamera2 import Picamera2
from libcamera import controls
import time
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
turn_array = [1,-1,-1,1,-1,-1,1]
#Configure Ultrasonic Sensor
GPIO_TRIGGER = 40
GPIO_ECHO = 38
GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
# Core camera setup
picam2 = Picamera2() # Assigns camera variable
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # Sets auto focus mode
picam2.start() # Activates camera
# Disable scientific notation for clarity
np.set_printoptions(suppress=True)
# Load the model
model = load_model("keras_model.h5", compile=False)
# Load the labels
class_names = open("labels.txt", "r").readlines()
# Move forward motion class
class RobotMotion():
def _init_(self):
print("Motion initialized")
class MotionPublisher(Node):
def __init__(self):
super().__init__('motion_publisher')
self.cp = RobotMotion()
print('Creating publisher')
self.motion_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
print('Creating a callback timer')
timer_period = .4
self.timer = self.create_timer(timer_period, self.timer_callback)
self.i = 0
self.current_speed = float(0) # Default Speed
def timer_callback(self):
#speed = float(2)
msg = Twist()
msg.linear.x = self.current_speed
self.motion_publisher.publish(msg)
def measure_distance():
# Set Trigger to HIGH
GPIO.output(GPIO_TRIGGER, True)
time.sleep(0.00001)
GPIO.output(GPIO_TRIGGER, False)
start_time = time.time()
stop_time = time.time()
# Save StartTime
while GPIO.input(GPIO_ECHO) == 0:
start_time = time.time()
# Save time of arrival
while GPIO.input(GPIO_ECHO) == 1:
stop_time = time.time()
# Time difference between start and arrival
time_elapsed = stop_time - start_time
print(time_elapsed)
# Speed of sound in air (343 meters per second) and 100 for conversion to centimeters
distance_cm = round((time_elapsed * 34300) / 2, 2)
print(distance_cm)
time.sleep(0.1)
return distance_cm
class RotateRobot(Node):
def __init__(self):
super().__init__('rotate_robot')
self.action_client = ActionClient(self, RotateAngle, '/rotate_angle')
def send_goal(self, angle, max_speed):
goal_msg = RotateAngle.Goal()
goal_msg.angle = angle
goal_msg.max_rotation_speed = max_speed
self.action_client.wait_for_server()
self._send_goal_future = self.action_client.send_goal_async(goal_msg)
self._send_goal_future.add_done_callback(self.goal_response_callback)
def goal_response_callback(self, future):
goal_handle = future.result()
if not goal_handle.accepted:
self.get_logger().info('Goal rejected :(')
return
self.get_logger().info('Goal accepted :)')
self._get_result_future = goal_handle.get_result_async()
self._get_result_future.add_done_callback(self.get_result_callback)
def get_result_callback(self, future):
#result = future.result().result
#self.get_logger().info(f'Final rotation: {result.final_angle}')
pass
def main(args = None):
rclpy.init(args=args)
rotate_robot = RotateRobot()
motion = MotionPublisher()
print('Callbacks are called')
try:
while True:
# Grab the picam's image.
image = picam2.capture_array("main")
# Show the image in a window
#cv2.imshow("Image", image)
# Assuming image has shape (height, width, channels)
image_3_channel = image[:, :, :3] # Select only the first three channels (RGB)
# Resize the image
image_resized = cv2.resize(image_3_channel, (224, 224),
interpolation=cv2.INTER_AREA)
#print("resized")
height, width, channels = image.shape
#print("Image Size: {} x {} pixels".format(width, height))
#print("channels: ", channels)
# Make the image a numpy array and reshape it to the models input shape.
image = np.asarray(image_resized, dtype=np.float32).reshape(1, 224, 224, 3)
#print("reshaped")
# Normalize the image array
image = (image / 127.5) - 1
# Predicts the model
prediction = model.predict(image)
index = np.argmax(prediction)
class_name = class_names[index]
confidence_score = prediction[0][index]
# Distance Calculations
distCM = measure_distance()
distFrmBumper = distCM - 2
distance = distFrmBumper/2.54
# Print prediction and confidence score and distance
#print("Distance:", distance, "in")
if distance <= 6:
#print("Object detected.. stopping")
motion.current_speed = float(0)
print("Stopped")
# Print class name and confidence score
print("Detected object:", class_name, "with confidence:", confidence_score)
# Rotate
# Rotate
turn_direction = turn_array[index]
rotate_robot.send_goal(1.57*turn_direction, 0.5)
rclpy.spin_once(rotate_robot)
else:
motion.current_speed = float(.1)
rclpy.spin_once(motion)
# Listen to the keyboard for presses.
keyboard_input = cv2.waitKey(1)
# 27 is the ASCII for the esc key on your keyboard.
if keyboard_input == 27:
break
except KeyboardInterrupt:
print('\nCaught Keyboard Interrupt')
finally:
motion.destroy_node()
rotate_robot.destroy_node()
GPIO.cleanup()
cv2.destroyAllWindows()
picam2.stop() # Deactivates cameraa
print("Done")
print('shutting down')
rclpy.shutdown()
if __name__ == '__main__':
main()
