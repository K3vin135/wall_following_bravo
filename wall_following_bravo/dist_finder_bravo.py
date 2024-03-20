#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import time
import numpy as np
from numpy import cos, sin, pi,tan,arctan,arctan2

pared=90*2
angulo=30*2


class dist_finder_bravo(Node):

    def __init__(self):
        super().__init__('dist_finder_bravo')
        self.publisher_error_msg = self.create_publisher(Float32, 'error_msg', 10)

        # subscriptor
        self.scan_sub= self.create_subscription(
            LaserScan,
            '/scan',
            self.function_callback,
            10)
        self.get_logger().info('HELLOOOOO')


    def getRange(self,data):

        #if (data[pared]+data[pared-angulo])/2<(data[pared+360]+data[pared+360+angulo])/2:
            self.get_logger().info('DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD')

            a=data[pared-angulo]
            b=data[pared] 


            theta=np.deg2rad(angulo/2)
            Desired_distance = 0.2
            alfa=arctan2((a*cos(theta)-b),a*sin(theta))
            AB=b*cos(alfa)

            AC=2
            CD=AB+sin(alfa)*AC
            
            error=Desired_distance-CD
            
            return -error
        # else:
        #     self.get_logger().info('IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII')
        #     a=data[pared+360+angulo]
        #     b=data[pared+360]

        #     theta=np.deg2rad(60)
        #     Desired_distance = 0.5
        #     alfa=arctan2((a*cos(theta)-b),a*sin(theta))
        #     AB=b*cos(alfa)

        #     AC=2
        #     CD=AB+sin(alfa)*AC
            
        #     error=Desired_distance-CD
            
        #     return error

    def function_callback(self,msg):
        self.get_logger().info('asdd')
        data=msg.ranges
        data=np.array(data)
        data=np.where(data==np.inf, 10.0, data)
        print(data.size)
        error= self.getRange(data)

        new_error= float(error)
        float32_msg = Float32()
        float32_msg.data = new_error

        self.publisher_error_msg.publish(float32_msg)

def main():
    rclpy.init()
    node = dist_finder_bravo()
    rclpy.spin(node)
    
    node.destroy_node()

