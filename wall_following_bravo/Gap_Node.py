#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import numpy as np
from numpy import cos, sin, pi,tan,arctan,arctan2
import matplotlib.pyplot as plt
import sys
import csv
import time 

class Gap_Node(Node):
    def __init__(self):
        super().__init__('gap_node')
        self.pos_subs = self.create_subscription(LaserScan,'scan',self.gap_callback,10)
        self.vel_pub = self.create_publisher(Twist,'cmd_route',10)
        self.steering_angle = 0

    def posicion_maxima_secuencia(self,arr):
        # Convertir la lista a un arreglo NumPy
        arr = np.array(arr)

        # Encontrar subarreglos donde arr no es cero
        non_zero_subarrays = np.split(arr, np.where(arr == 0)[0])
        subarreglo_sin_ceros = [non_zero_subarrays[non_zero_subarrays != 0] for non_zero_subarrays in non_zero_subarrays]

        # Filtrar subarreglos no vacíos
        subarreglo_sin_ceros = [subarray for subarray in subarreglo_sin_ceros if len(subarray) > 0]
        search = max(subarreglo_sin_ceros,key=len) # Busca el GAP más grande
        posicion = arr.tolist().index(search[0]) # indica la posición del GAP más grande
        posicionF = posicion + len(search) - 1

        return posicion, posicionF, search


    def gap_callback(self, msg):
        kp = 8 
        #kd = 18  
        max_steering = 0.5
        min_steering = -0.5 
        forward_velocity = 0.3

        array = msg.ranges

        min = np.min(array)

        #Replace min values to 0
        new_array = np.where((array < min+0.2), 0, array)

        #Obtain max gap, distance to max gap and angle between lenght of the gap
        pos_max, pos_Fin, max_array = self.posicion_maxima_secuencia(new_array)
        
        angle = pos_max - pos_Fin 

        A = max_array[0]
        B = max_array[-1]

        C = np.sqrt(np.power(A,2)+np.power(B,2)-2*A*B*cos(np.deg2rad(angle)))

        # Following middle gap point
        piece = (len(max_array)-1)//2

        set_point = array.tolist().index(max_array[piece])
        print(set_point)

        error = set_point - 179 


        steering_correction = -(kp * error )
        self.steering_angle = self.steering_angle - steering_correction

                # Check for saturation on steering
        if self.steering_angle > max_steering:
            self.steering_angle = max_steering
            
        elif self.steering_angle < min_steering:
            self.steering_angle = min_steering
            
        # Crear y publicar el mensaje de velocidad de comando
        cmd_vel_gap = Twist()
        cmd_vel_gap.linear.x = forward_velocity
        cmd_vel_gap.angular.z = self.steering_angle
        self.vel_pub.publish(cmd_vel_gap)


        
        


def main():
    rclpy.init()
    node = Gap_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()