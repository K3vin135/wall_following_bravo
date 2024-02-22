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
import time as timexxx
from rosgraph_msgs.msg import Clock


class Gap_Node(Node):
    def __init__(self):
        super().__init__('gap_node')
        self.pos_subs = self.create_subscription(LaserScan,'scan',self.gap_callback,10)
        self.vel_pub = self.create_publisher(Twist,'cmd_route',10)
        self.steering_angle = 0
        self.prev_T=0
        self.deltaT=1

        # subscriptor
        self.error_sub= self.create_subscription(
        Clock,
        'clock',
        self.time_calculation,
        10)

    def posicion_maxima_secuencia(self,arr):
        # Convertir la lista a un arreglo NumPy
        arr = np.array(arr)

        # Dividir el arreglo en subarreglos cada que encuentre un 0
        non_zero_subarrays = np.split(arr, np.where(arr == 0)[0])
        # Encontrar subarreglos donde arr no es cero
        subarreglo_sin_ceros = [non_zero_subarrays[non_zero_subarrays != 0] for non_zero_subarrays in non_zero_subarrays]

        # Filtrar subarreglos no vacíos
        subarreglo_sin_ceros = [subarray for subarray in subarreglo_sin_ceros if len(subarray) > 0]

        #Hayar el subarregklo con el promedio más grande
        promedios = np.array([np.mean(arreglo) for arreglo in subarreglo_sin_ceros])
        max_promedio_index = np.argmax(promedios)

        posicion = np.where(arr == subarreglo_sin_ceros[max_promedio_index][0])[0][0] # Buscamos donde esta el arreglo con mayor promedio 
        posicionF = posicion + len(subarreglo_sin_ceros[max_promedio_index]) - 1


        #search = max(subarreglo_sin_ceros,key=len) # Busca el GAP más grande
        return posicion, posicionF, subarreglo_sin_ceros[max_promedio_index]
    
    def time_calculation(self,msg):
        T=msg.clock
        T_nanosec=T.nanosec
        T_total=T_nanosec*10**(-9)
        self.deltaT=T_total-self.prev_T
        self.prev_T=T_total
        return self.deltaT


    def gap_callback(self, msg):
        time=self.deltaT
        kp = 3 
        kd = 10.0  
        max_steering = 3.0
        min_steering = -3.0 
        forward_velocity = 1.0

        array = msg.ranges[(89+10):(269-10)]
        array=np.array(array)
        array[:30] *= 0.4  
        array[130:160] *= 0.4

        array=np.where(array==np.inf, 20, array)

        if array[159]>=10 or array[158]>=10 or array[157]>=10 or array[156]>=10 or array[155]>=10 or array[154]>=10 or array[153]>=10 or array[152]>=10 or array[151]>=10 or array[150]>=10:
            array[159]=3
            array[158]=3
            array[157]=3
            array[156]=3
            array[155]=3
            array[154]=3
            array[153]=3
            array[152]=3
            array[151]=3
            array[150]=3
            array[149]=3
            array[148]=3
            array[147]=3
            array[146]=3

                       
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
        

        error = set_point - (89-10) 

        print(array[0])
        steering_correction = -(kp * error + kd * error/time)
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
