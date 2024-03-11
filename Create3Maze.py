import sys 
import rclpy
from rclpy.node import Node
import random
import requests # you may need to run 'pip install requests' to install this library
import json 
import time
from geometry_msgs.msg import Twist

'''
These statements import iRobot Create®3 messages and actions.
'''

class RobotMotion():
    def _init_(self):
        print("Motion initialized")

class MotionPublisher(Node):
    def _init_(self):    
        super()._init_('motion_publisher')

        self.cp = RobotMotion()

        print('Creating publisher')
        self.motion_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        '''
        The timer allows the callback to execute every 2 seconds, with a counter iniitialized.
        '''
        print('Creating a callback timer') 
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.previous_rotation = 0
        self.previous_translational = 0

    def timer_callback(self):
        current_time = self.get_clock().now()

        r = requests.get(url = URL, headers = Headers, params = {})
        data = r.json()
        new_rotation = data['records'][1]['fields']['Value']
        new_translation = data['records'][2]['fields']['Value']

        msg = Twist()
        msg.linear.x = float(data['records'][2]['fields']['Value'])
        msg.angular.z = float(data['records'][1]['fields']['Value'])
        #self.publisher.publish(msg)
        if (new_rotation != self.previous_rotation) or (new_translation != self.previous_translation):
            print("New Values- Rotation: ", new_rotation, " Translation: ", new_translation)
        else:
            print("No change")

        self.previous_rotation = new_rotation
        self.previous_translational = new_translation
        self.publisher_.publish(msg)
    def reset(self):
        '''
        This function releases contriol of the lights and "gives" it back to the robot. 
        '''
        print('Resetting motion')

        
''' This function makes a get request to the airtable API which will tell us how fast to spin the wheels'''

''' Put the URL for your Airtable Base here'''
''' Format: 'https://api.airtable.com/v0/BaseID/tableName '''
URL = 'https://api.airtable.com/v0/applK0NRaebYim73c/Control_Table'


''' Format: {'Authorization':'Bearer Access_Token'}
Note that you need to leave "Bearer" before the access token '''
Headers = {'Authorization':'Bearer patdq1umq6dz00Wjg.1fa65436b75bc5e22d7d04d39029973f64a8cc0c2057984e84e58a1947f50132'}


'''
The get request data comes in as a json package. We will convert this json package to a python dictionary so that it can be parsed
'''

def main(args = None):
    rclpy.init(args=args)
    motion = MotionPublisher()
    print('Callbacks are called')
    try:
        rclpy.spin(motion)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    finally:
        print("Done")
        motion.reset()
        motion.destroy_node()
        print('shutting down')
        rclpy.shutdown()
