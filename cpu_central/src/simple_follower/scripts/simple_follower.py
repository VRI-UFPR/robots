from object_recognition.msg import ModelResults
from rclpy.node import Node

# Define constants for directions and thresholds
PERSON_CLASS = 'person'
LEFT = 'left'
RIGHT = 'right'
FORWARD = 'forward'
STOP = 'stop'
CENTER_MIN = 3030
CENTER_MAX = 340
IMAGE_CENTER = 320

class simple_follower(Node):

    def __init__(self):
        super().__init__('simple_follower')

            # subscription to model results
        self.subscription = self.create_subscription(ModelResults, '/model_results', self.calc_movement, 10)


    def move_robot(self, speed, direction):
        # geometry_msgs/Twist Message
        # alterar o x do primeiro vetor e o z do segundo.
        print(f"Speed: {speed}, Direction: {direction}")

    def calc_movement(self, data):
        # checks if the object is a person, if not, stops the robot
        # the robot follows the first person detected in the image
        for r in data.model_results:
            if r.class_name == PERSON_CLASS:
                x = (r.top + r.bottom) / 2
                direction = self.get_direction(x)
                speed = self.calculate_speed(x)

                self.move_robot(speed, direction)
                return  # Stop after finding the first person
        
        # If no person is detected, stop the robot
        self.move_robot(0, STOP)

    def get_direction(self, x):
        if x < CENTER_MIN:
            return LEFT
        elif x > CENTER_MAX:
            return RIGHT
        return FORWARD

    def calculate_speed(self, x):
        return abs(x - IMAGE_CENTER) / IMAGE_CENTER
        # deve estar entre 0 20 ou 30
        


