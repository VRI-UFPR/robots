#!/usr/bin/env python3

# =============================================================================
#  Header
# =============================================================================

import math
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

# =============================================================================
#  MyRobotDriver
# =============================================================================

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel')
        self.__right_motor = self.__robot.getDevice('right wheel')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        self.left_encoder = self.__robot.getDevice('left wheel sensor')
        self.right_encoder = self.__robot.getDevice('right wheel sensor')
        self.left_encoder.enable(100)
        self.right_encoder.enable(100)

        self.last_encoder_left = 0
        self.last_encoder_right = 0

        self.WHEEL_RADIUS = 0.033  # Raio da roda em metros
        self.WHEEL_BASE = 0.16     # Distância entre as rodas em metros
        self.ENCODER_RESOLUTION = 159.23  # Pulsos por revolução

        self.last_time = self.__node.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.tf_broadcaster = TransformBroadcaster(self.__node)


    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)
        self.update_odometry()

    def update_odometry(self):
        """Calcula e publica a odometria com base nos encoders das rodas"""
        # Lê os valores atuais dos encoders
        encoder_left = self.left_encoder.getValue()
        encoder_right = self.right_encoder.getValue()
        
        # Calcula a diferença desde a última leitura
        delta_left = encoder_left - self.last_encoder_left
        delta_right = encoder_right - self.last_encoder_right
        
        # Atualiza os valores anteriores
        self.last_encoder_left = encoder_left
        self.last_encoder_right = encoder_right
        
        # Converte pulsos do encoder para distância percorrida (em metros)
        # distance_left = (2 * math.pi * self.WHEEL_RADIUS * delta_left) / self.ENCODER_RESOLUTION
        # distance_right = (2 * math.pi * self.WHEEL_RADIUS * delta_right) / self.ENCODER_RESOLUTION
        
        distance_left = delta_left
        distance_right = delta_right

        # print(distance_left, distance_right)

        # Calcula o deslocamento linear e angular
        linear = (distance_right + distance_left) / 2.0
        angular = (distance_right - distance_left) / self.WHEEL_BASE
        
        # Tempo decorrido desde a última atualização
        current_time = self.__node.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        print(self.theta, linear, angular, math.sin(self.theta) )
        # Atualiza a posição e orientação
        # if self.theta == 0.0:
        #    radius = linear / angular
        #    self.x += radius * (math.sin(self.theta))
        #    self.y -= radius * (math.cos(self.theta))
        #    # self.theta += angular
        # else:
        self.x += linear * math.cos(self.theta)
        self.y += linear * math.sin(self.theta)
        
        # Normaliza o ângulo entre -pi e pi
        # self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Cria e publica a mensagem de odometria
        # odom_msg = Odometry()
        # odom_msg.header.stamp = current_time.to_msg()
        # odom_msg.header.frame_id = 'odom'
        # odom_msg.child_frame_id = 'base_link'
        
        # Posição
        # odom_msg.pose.pose.position.x = self.x
        # odom_msg.pose.pose.position.y = self.y
        # odom_msg.pose.pose.position.z = 0.0
        
        # Orientação (convertida para quaternion)
        # odom_msg.pose.pose.orientation.x = 0.0
        # odom_msg.pose.pose.orientation.y = 0.0
        # odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        # odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocidade
        # odom_msg.twist.twist.linear.x = linear / dt if dt > 0 else 0.0
        # odom_msg.twist.twist.angular.z = angular / dt if dt > 0 else 0.0
        # self.odom_publisher.publish(odom_msg)
        
        # Publica a transformada TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

# =============================================================================
#  Documentation
# =============================================================================

# O codigo já publica os dispotivos do robo

# Topicos
"""
/cmd_vel
/parameter_events
/pioneer/RPlidar_A2
/pioneer/RPlidar_A2/point_cloud
/pioneer/green_led
/pioneer/kinect_color/camera_info
/pioneer/kinect_range/camera_info
/pioneer/kinect_range/image
/pioneer/kinect_range/point_cloud
/pioneer/lower_yellow_led
/pioneer/red_led_1
/pioneer/red_led_2
/pioneer/red_led_3
/pioneer/so0
/pioneer/so1
/pioneer/so10
/pioneer/so11
/pioneer/so12
/pioneer/so13
/pioneer/so14
/pioneer/so15
/pioneer/so2
/pioneer/so3
/pioneer/so4
/pioneer/so5
/pioneer/so6
/pioneer/so7
/pioneer/so8
/pioneer/so9
/pioneer/white_led
/pioneer/yellow_led
/remove_urdf_robot
/rosout
"""