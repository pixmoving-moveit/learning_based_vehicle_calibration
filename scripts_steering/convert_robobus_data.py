from typing import List
import rclpy
from rclpy.context import Context
import rclpy.node
import pandas as pd
import math
from collections import deque
from statistics import mean

# from pix_hooke_driver_msgs.msg import V2aBrakeStaFb, V2aDriveStaFb, V2aSteerStaFb
from pix_robobus_driver_msgs.msg import SteeringReport, ThrottleReport, BrakeReport, VcuReport
from autoware_auto_vehicle_msgs.msg import VelocityReport
from rclpy.parameter import Parameter
from tier4_vehicle_msgs.msg import ActuationCommand
from sensor_msgs.msg import Imu
from can_msgs.msg import Frame
from std_msgs.msg import Float32

class ConvertMessage(rclpy.node.Node):
    
    def __init__(self):
        super().__init__('convert_robobus_message')
        self.brake_value = 0.0
        self.throttle_value = 0.0
        self.steering_value = 0.0
        self.velocity = 0.0
        self.create_subscription(BrakeReport, '/pix_robobus/brake_report', self.brake_topic_callback, 1)
        self.create_subscription(ThrottleReport, '/pix_robobus/throttle_report', self.drive_topic_callback, 1)
        self.create_subscription(SteeringReport, '/pix_robobus/steering_report', self.steer_topic_callback, 1)
        self.create_subscription(VcuReport, '/pix_robobus/vcu_report', self.velocity_topic_callback, 1)
        self.actuation_pub = self.create_publisher(ActuationCommand, 'actuation_input', 10)
        self.velocity_pub = self.create_publisher(VelocityReport, 'velocity_input', 10)


        self.timer = self.create_timer(0.02, self.time_callback)

    def brake_topic_callback(self, msg):
        self.brake_value = float(msg.brake_pedal_actual)

    def drive_topic_callback(self, msg):
        self.throttle_value = float(msg.dirve_throttle_pedal_actual)

    def steer_topic_callback(self, msg):
        value = float(msg.steer_angle_actual)
        self.steering_value = value / 450

    def velocity_topic_callback(self, msg):
        self.velocity = float(msg.vehicle_speed)


    def time_callback(self):
        actua = ActuationCommand()
        actua.accel_cmd = self.throttle_value
        actua.brake_cmd = self.brake_value
        actua.steer_cmd = self.steering_value
        self.actuation_pub.publish(actua)

        velocity = VelocityReport()
        velocity.longitudinal_velocity = self.velocity
        self.velocity_pub.publish(velocity)





def main():
      rclpy.init()
      node = ConvertMessage()
      rclpy.spin(node)




if __name__ == '__main__':
      main()

