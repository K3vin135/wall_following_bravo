#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rclpy.clock import ROSClock

class Control_bravo(Node):

    def __init__(self):
        super().__init__('control_bravo')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_ctrl', 10)
        self.prev_error = 0
        self.steering_angle = 0
        self.prev_time = self.get_clock().now()
        self.error = 0

        # Suscriptor
        self.error_sub = self.create_subscription(
            Float32,
            'error_msg',
            self.error_callback,
            10)

        # Temporizador para el control PD
        control_frequency = 10.0  # 10 Hz
        self.timer = self.create_timer(1.0 / control_frequency, self.control_callback)

    def error_callback(self, msg):
        # Almacenar el error más reciente
        self.error = msg.data

    def control_callback(self):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.prev_time).nanoseconds * 10**(-9)

        if delta_time == 0:
            return

        # PD control
        kp = 0.08
        kd = 1
        max_steering = 0.3
        min_steering = -0.3
        forward_velocity = 0.2

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
        #cmd_vel.linear.x = forward_velocity
        cmd_vel.linear.x = 0.2
        cmd_vel.angular.z = self.steering_angle
        self.cmd_pub.publish(cmd_vel)

        # Actualizar valores para la próxima iteración
        self.prev_error = self.error
        self.prev_time = current_time

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