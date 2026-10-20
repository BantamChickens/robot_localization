import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

'''Goals
    - Constant forward motion and randomly generate angles (but not too sharp)
    - Collison and clipping avoidance (if bump into obstacle, stop, back up, turn and retry)
   Resources
    - Reference drive square (maybe similar + random angles)''' 

class MotionModel(Node):
    def __init__(self):
        super().__init__('motion')
        
        #----------Set cordinate frames and configuration parameters------------------------------------------------
        # TODO: define additional constants if needed

        #-------------------Publishers and Subscribers------------------------------------------------
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        #-------------------Set up inital filter states------------------------------------------------
        # knowing incoming timestamp info helps us set the timestamp of our map -> odom transform correctly
        self.last_scan_timestamp = None # keeps track of the timestamps coming from bag files  
        self.scan_to_process = None     # current scan that run_loop should process
        
        self.particle_cloud = []        # particle cloud will go here
        self.current_odom_xy_theta = []
        
        #-------------------Timer------------------------------------------------

        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

