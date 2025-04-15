from math import atan2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu 
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.reset_state()
        self.cmd_vel_msg = Twist()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.direction1= None
        self.direction2= None
        self.scan_subscriber = self.create_subscription(String, '/obstacle_direction', self.object_callback, 10)
        self.pit_subscriber = self.create_subscription(String, '/pit_direction', self.pit_callback, 10)
        self.imu = self.create_subscription(Float64, '/pitch', self.imu_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.2, self.avoid_obstacle)
    def reset_state(self):
        """Reset variables for handling new obstacles and pits."""
        self.left_obstacle = False
        self.front_obstacle = False
        self.right_obstacle = False
        self.left_pit = False
        self.front_pit = False
        self.right_pit = False
        self.record_time_diagonal = None
        self.target_yaw = None
        self.yaw = None
        self.obstacle = False
        self.pit = False
        self.prev_direction="Front"
        self.record_time_parallel_path = None
    def imu_callback(self, msg):
        q = msg.orientation
        self.yaw = q * (math.pi / 180.0)
    def pit_callback(self, msg):
        self.direction2 = msg.data
        if self.direction2 == 'Front':
            self.front_pit= False
            self.left_pit= False
            self.right_pit = False
            self.pit=False
        elif self.direction2 == 'Front_Left' or self.direction2 == 'Front_Right':
            self.front_pit = True
            self.left_pit = False
            self.right_pit = False
                 
        if self.front_pit and self.target_yaw is None:
            self.target_yaw = self.yaw
            self.record_time_diagonal = time.time()
    def object_callback(self, msg):
        self.direction1 = msg.data
        if self.direction1 == 'Front':
            self.front_obstacle = False
            self.left_obstacle = False
            self.right_obstacle = False
            self.obstacle=False
        elif self.direction1 == 'Front_Left' or self.direction1 == 'Front_Right':
            self.front_obstacle = True
            self.left_obstacle = False
            self.right_obstacle = False
        elif self.direction1 == 'Left':
            self.front_obstacle = False
            self.left_obstacle = True
            self.right_obstacle = False
        elif self.direction1 == 'Right':
            self.front_obstacle = False
            self.left_obstacle = False
            self.right_obstacle = True
        
        if self.front_obstacle and self.target_yaw is None:
            self.target_yaw = self.yaw
            self.record_time_diagonal = time.time()
    def avoid_obstacle(self):
        if not self.front_obstacle and not self.obstacle and not self.front_pit and not self.pit:
            self.get_logger().info("no obstacle and going front")
            self.linear_velocity = 0.5
            self.angular_velocity = 0.0
            self.prev_direction="Front"
        elif self.front_obstacle or self.front_pit:
            self.get_logger().info("Front obstacle detected, deciding rotation direction")
            if self.direction1 == 'Front_Left' or self.direction2 == 'Front_Left':
                if self.prev_direction=="Front_Right":
                    self.get_logger().info("Rotating left (more obstacle on the right)")
                    self.linear_velocity = 0.0
                    self.angular_velocity = 0.5
                    self.rotating_angle = self.angular_velocity
                    self.prev_direction="Front_Right"
                else:
                    self.get_logger().info("Rotating right (more obstacle on the left)")
                    self.linear_velocity = 0.0
                    self.angular_velocity = -0.5
                    self.rotating_angle = self.angular_velocity
                    self.prev_direction="Front_Left"
            elif self.direction1 == 'Front_Right' or self.direction2 == 'Front_Right':
                if self.prev_direction=="Front_Left":
                    self.get_logger().info("Rotating right (more obstacle on the left)")
                    self.linear_velocity = 0.0
                    self.angular_velocity = -0.5
                    self.rotating_angle = self.angular_velocity
                    self.prev_direction="Front_Left"
                else :
                    self.get_logger().info("Rotating left (more obstacle on the right)")
                    self.linear_velocity = 0.0
                    self.angular_velocity = 0.5
                    self.rotating_angle = self.angular_velocity
                    self.prev_direction="Front_Right" 
            self.obstacle = True
            if self.target_yaw is None:
                self.target_yaw = self.yaw
                self.record_time_diagonal = time.time()
        elif self.obstacle and not self.front_obstacle:
            if self.record_time_diagonal is not None and time.time() - self.record_time_diagonal < 2.0:
                self.get_logger().info("obstacle detected and moving diagonally")
                self.linear_velocity = 0.5
                self.angular_velocity = 0.0
            elif abs(self.yaw - self.target_yaw) > 0.3:
                self.get_logger().info("rotating to parallel path")
                self.record_time_parallel_path = time.time()
                self.linear_velocity = 0.0
                self.angular_velocity = -self.rotating_angle
            elif self.record_time_parallel_path is not None and time.time() - self.record_time_parallel_path < 2.0:
                self.get_logger().info("moving in parallel path")
                self.linear_velocity = 0.5
                self.angular_velocity = 0.0
            else:
                self.get_logger().error('Resetting obstacle avoidance')
                self.reset_state()
        elif self.left_obstacle or self.right_obstacle:
            self.get_logger().info("left or right obstacle detected and moving forward")
            self.linear_velocity = 0.5
            self.angular_velocity = 0.0

        self.cmd_vel_msg.linear.x = self.linear_velocity
        self.cmd_vel_msg.angular.z = self.angular_velocity
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoider = ObstacleAvoider()
    rclpy.spin(obstacle_avoider)
    obstacle_avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
