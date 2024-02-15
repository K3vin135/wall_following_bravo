#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
from numpy import cos, sin, pi,tan,arctan,arctan2

from rosgraph_msgs.msg import Clock

class Control_bravo(Node):

    def __init__(self):
        super().__init__('control_bravo')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_ctrl', 10)
        self.prev_error = 0
        self.steering_angle = 0
        self.prev_T=0
        self.deltaT=1


        # subscriptor
        self.error_sub= self.create_subscription(
            Clock,
            'clock',
            self.time_calculation,
            10)
        
        self.error_sub= self.create_subscription(
            Float32,
            'error_msg',
            self.function_callback,
            10)
        
    def time_calculation(self,msg):
        T=msg.clock
        T_nanosec=T.nanosec
        T_total=T_nanosec*10**(-9)
        self.deltaT=T_total-self.prev_T
        self.prev_T=T_total
        return self.deltaT

    def function_callback(self,msg):

        time=self.deltaT
        self.get_logger().info('drozo')
        kp = 0.8 
        kd = 3.212  
        max_steering = 3.0
        min_steering = -3.0 
        forward_velocity = 1.1

        # PD control
        error = msg.data

        delta_error = error - self.prev_error

        self.prev_error = error

        steering_correction = -(kp * error + kd * delta_error/time)
        self.steering_angle = self.steering_angle - steering_correction

        # Check for saturation on steering
        if self.steering_angle > max_steering:
            self.steering_angle = max_steering
            
        elif self.steering_angle < min_steering:
            self.steering_angle = min_steering
            
        # Crear y publicar el mensaje de velocidad de comando
        cmd_vel = Twist()
        cmd_vel.linear.x = forward_velocity
        cmd_vel.angular.z = self.steering_angle
        self.cmd_pub.publish(cmd_vel)

def main():
    rclpy.init()
    node = Control_bravo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        


