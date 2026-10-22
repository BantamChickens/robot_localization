import rclpy
from rclpy.node import Node
from time import sleep
from geometry_msgs.msg import Twist
import numpy as np
import math
import random


'''Goals
    - Constant forward motion and randomly generate angles (but not too sharp)
    - Collison and clipping avoidance (if bump into obstacle, stop, back up, turn and retry)
   Resources
    - Reference drive square (maybe similar + random angles)''' 


class MotionModel(Node):
    """A class that implements a node to pilot a robot in a square.
    """

    def __init__(self):
        super().__init__('motion')
        # create a thread to handle long-running component
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.max_angle = 90
        self.angle_step = 15
        self.angular_vel = 0.3

    def drive(self, linear, angular):
        """Drive with the specified linear and angular velocity.

        Args:
            linear (_type_): the linear velocity in m/s
            angular (_type_): the angular velocity in radians/s
        """        
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)
    
    def degrees2radians(self, num_list):
        return [math.radians(angle) for angle in num_list]
    
    def turn(self):
        """Execute a 90 degree left turn
        """
        angles_list = list(
            range(
                -self.max_angle,
                self.max_angle + 1,
                self.angle_step,
            )
        )

        angles = self.degrees2radians(angles_list)
        selected_angle = random.choice(angles)

        if selected_angle == 0.0:
            return
        else:
            a_vel = math.copysign(self.angular_vel, selected_angle)

        self.drive(linear=0.0, angular=a_vel)
        sleep(abs(selected_angle / a_vel))
        self.drive(linear=0.0, angular=0.0)

    def drive_forward(self, distance):
        """Drive straight for the specified distance.

        Args:
            distance (_type_): the distance to drive forward.  Only positive
            values are supported.
        """
        forward_vel = 0.1
        self.drive(linear=forward_vel, angular=0.0)
        sleep(distance / forward_vel)
        self.drive(linear=0.0, angular=0.0)

    def run_loop(self):
        """Executes the main logic for driving the square.  This function does
        not return until the square is finished or the estop is pressed.
        """
        # the first message on the publisher is often missed
        self.drive(0.0, 0.0)
        sleep(1)
        self.drive_forward(0.5)
        self.turn()

def main(args=None):
    rclpy.init(args=args)
    node = MotionModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

