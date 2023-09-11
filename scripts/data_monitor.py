#! /usr/bin/env python3
import rclpy
import rclpy.node
from pix_hooke_driver_msgs.msg import V2aBrakeStaFb, V2aDriveStaFb, V2aSteerStaFb
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from can_msgs.msg import Frame


class DataMonitor(rclpy.node.Node):
    def __init__(self):
        self.brake_timestamp = 0.0
        self.throttle_timestamp = 0.0
        self.velocity_timestamp = 0.0
        self.steering_timestamp = 0.0
        self.pitch_timestamp = 0.0
        self.imu_timestamp = 0.0
        self.can_timestamp = 0.0
        super().__init__('data_monitor')

        self.timer = self.create_timer(1, self.timer_callback)
        self.create_subscription(V2aBrakeStaFb, '/pix_hooke/v2a_brakestafb', self.brake_topic_callback, 10)
        self.create_subscription(V2aDriveStaFb, '/pix_hooke/v2a_drivestafb', self.drive_topic_callback, 10)
        self.create_subscription(V2aSteerStaFb, '/pix_hooke/v2a_steerstafb', self.steer_topic_callback, 10)
        self.create_subscription(Float32, '/gnss/chc/pitch', self.pitch_topic_callback, 10)
        self.create_subscription(Imu, '/gnss/chc/imu', self.imu_topic_callback, 10)
        self.create_subscription(Frame, '/from_can_bus', self.can_topic_callback, 10)


    def can_topic_callback(self, msg):
        # self.brake_timestamp = self.get_clock().now().nanoseconds
        self.can_timestamp = int(self.get_clock().now().nanoseconds/1000000)
        # self.get_logger().info("Hello: %s", self.brake_timestamp)
        # print(self.brake_timestamp)

    def brake_topic_callback(self, msg):
        # self.brake_timestamp = self.get_clock().now().nanoseconds
        self.brake_timestamp = int(self.get_clock().now().nanoseconds/1000000)
        # self.get_logger().info("Hello: %s", self.brake_timestamp)
        # print(self.brake_timestamp)

    def drive_topic_callback(self,msg):
        timestamp = int(self.get_clock().now().nanoseconds/1000000)
        self.throttle_timestamp = timestamp
        self.velocity_timestamp = timestamp
    
    def steer_topic_callback(self, msg):
        self.steering_timestamp = int(self.get_clock().now().nanoseconds/1000000)

    def pitch_topic_callback(self, msg):
        self.pitch_timestamp = int(self.get_clock().now().nanoseconds/1000000)
    
    def imu_topic_callback(self, msg):
        self.imu_timestamp = int(self.get_clock().now().nanoseconds/1000000)

        


    def timer_callback(self):
        self.get_logger().info("data monitor checking")
        timestamp = int(self.get_clock().now().nanoseconds/1000000)
        
        can_timegap = timestamp - self.can_timestamp
        brake_timegap = timestamp - self.brake_timestamp
        throttle_timegap = timestamp - self.throttle_timestamp
        steering_timegap = timestamp - self.steering_timestamp
        pitch_timegap = timestamp - self.pitch_timestamp
        imu_timegap = timestamp - self.imu_timestamp

        if self.can_timestamp == 0:
            self.get_logger().error("can topic is not publish")
        elif can_timegap > 1000:
            self.get_logger().error("can topic is not alive")
        else:
            self.get_logger().debug("can topic is good")

        if self.brake_timestamp == 0:
            self.get_logger().error("brake topic is not publish")
        elif brake_timegap > 1000:
            self.get_logger().error("brake topic is not alive")
        else:
            self.get_logger().debug("brake topic is good")

        if self.throttle_timestamp == 0:
            self.get_logger().error("throttle topic is not publish")
        elif throttle_timegap > 1000:
            self.get_logger().error("throttle topic is not alive")
        else:
            self.get_logger().debug("throttle topic is good")

        if self.steering_timestamp == 0:
            self.get_logger().error("steering topic is not publish")
        elif steering_timegap > 1000:
            self.get_logger().error("steering topic is not alive")
        else:
            self.get_logger().debug("steering topic is good")
        
        if self.pitch_timestamp == 0:
            self.get_logger().error("pitch topic is not publish")
        elif pitch_timegap > 1000:
            self.get_logger().error("pitch topic is not alive")
        else:
            self.get_logger().debug("pitch topic is good")
        
        if self.imu_timestamp == 0:
            self.get_logger().error("imu topic is not publish")
        elif imu_timegap > 1000:
            self.get_logger().error("imu topic is not alive")
        else:
            self.get_logger().debug("imu topic is good")

    
def main():
    rclpy.init()
    node = DataMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()