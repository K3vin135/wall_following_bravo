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
from rclpy.clock import ROSClock
import time as timexxx
from rosgraph_msgs.msg import Clock
import time


class Gap_Node(Node):
    def __init__(self):
        super().__init__('gap_node')
        self.pos_subs = self.create_subscription(LaserScan,'/scan',self.gap_callback,10)
        self.cmd_pub = self.create_publisher(Twist,'cmd_route',10)
        self.steering_angle = 0
        self.prev_error = 0
        self.prev_time = self.get_clock().now()
        self.error = 0
        self.resolucion=2

        # Temporizador para el control PD
        control_frequency = 10.0  # 10 Hz
        self.timer = self.create_timer(1.0 / control_frequency, self.control_callback)


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
    


    def gap_callback(self, msg):
        

        array1=np.array(msg.ranges)
        
        arrayaux=array1[0:69*self.resolucion]
        arrayaux=np.flip(arrayaux)

        arrayaux2=array1[289*self.resolucion:359*self.resolucion]
        arrayaux2=np.flip(arrayaux2)


        array = np.concatenate([arrayaux,arrayaux2])
        array = np.flip(array)
        # array[:30] *= 0.7  
        # array[130:160] *= 0.7

        array=np.where(array==np.inf, 12, array)
        print(array[0])
        print(array[138])             
        min = np.min(array)

        #Replace min values to 0
        new_array = np.where((array < min+0.2), 0, array)

        #Obtain max gap, distance to max gap and angle between lenght of the gap
        pos_max, pos_Fin, max_array = self.posicion_maxima_secuencia(new_array)
        

        # Following middle gap point
        piece = (len(max_array))//2 - 1

        set_point = new_array.tolist().index(max_array[piece])
        print(max_array)

        self.error = set_point - 70*self.resolucion
        print("error: ", self.error)
        print("set_point: ", set_point)
        print("distancia: ", array[set_point])
      
    def control_callback(self):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.prev_time).nanoseconds * 10**(-9)

        if delta_time == 0:
            return

        # PD control
        kp = 0.06
        kd = 0.4
        max_steering = 0.3
        min_steering = -0.3
        forward_velocity = 0.3

        delta_error = self.error - self.prev_error

        steering_correction = -(kp * self.error + kd * delta_error / delta_time)
        self.steering_angle = self.steering_angle - steering_correction

        # Check for saturation on steering
        if self.steering_angle > max_steering:
            self.steering_angle = max_steering
        elif self.steering_angle < min_steering:
            self.steering_angle = min_steering

        # Publicar el mensaje de velocidad de comando
        cmd_vel = Twist()
        cmd_vel.linear.x = forward_velocity
        cmd_vel.angular.z = self.steering_angle
        self.cmd_pub.publish(cmd_vel)

        # Actualizar valores para la próxima iteración
        self.prev_error = self.error
        self.prev_time = current_time



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
