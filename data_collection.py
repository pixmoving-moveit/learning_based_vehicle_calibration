#/usr/bin/python

import rclpy
import rclpy.node
import pandas as pd
import math

from pix_hooke_driver_msgs.msg import V2aBrakeStaFb, V2aDriveStaFb, V2aSteerStaFb
from sensor_msgs.msg import Imu
from can_msgs.msg import Frame
from std_msgs.msg import Float32

from tqdm import tqdm

class primotest(rclpy.node.Node):

    def __init__(self):

        super().__init__('primo_test')

        self.i = 0
        self.j = 0
        self.h = 0
        self.a = 0
        self.b = 0
        self.c = 0
        self.ii = 0
        self.jj = 0
        self.hh = 0
        self.aa = 0
        self.bb = 0
        self.cc = 0
        self.vel = []
        self.cmd = []
        self.acc = []
        self.velb = []
        self.cmdb = []
        self.accb = []

        self.MAX_DATA = 3000
        self.SPEED_THRESHOLD = 2.7       # 10km/h
        self.STEERING_THRESHOLD = 40   # 2degrees
        self.THROTTLE_DEADZONE = 7
        self.BRAKE_DEADZONE = 7
 
        self.g = 9.8

        self.progress_bar1 = tqdm(total = self.MAX_DATA, desc = "Low speed: Throttle deadzone - 25 ")
        self.progress_bar2 = tqdm(total = self.MAX_DATA, desc = "Low speed: Throttle 25 - 35       ")
        self.progress_bar3 = tqdm(total = self.MAX_DATA, desc = "Low speed: Throttle > 35          ")
        self.progress_bar4 = tqdm(total = self.MAX_DATA, desc = "High speed: Throttle deadzone - 25")
        self.progress_bar5 = tqdm(total = self.MAX_DATA, desc = "High speed: Throttle 25 - 35      ")
        self.progress_bar6 = tqdm(total = self.MAX_DATA, desc = "High speed: Throttle > 35         ")
        self.progress_bar7 = tqdm(total = self.MAX_DATA, desc = "Low speed: Brake deadzone - 20    ")
        self.progress_bar8 = tqdm(total = self.MAX_DATA, desc = "Low speed: Brake 20 - 30          ")
        self.progress_bar9 = tqdm(total = self.MAX_DATA, desc = "Low speed: Brake > 30             ")
        self.progress_bar10 = tqdm(total = self.MAX_DATA, desc = "High speed: Brake deadzone - 20   ")
        self.progress_bar11 = tqdm(total = self.MAX_DATA, desc = "High speed: Brake 20 - 30         ")
        self.progress_bar12 = tqdm(total = self.MAX_DATA, desc = "High speed: Brake > 30            ")

        self.timer = self.create_timer(0.1, self.test_callback)
        self.create_subscription(Float32, '/gnss/chc/pitch', self.pitch_topic_callback, 10)
        self.create_subscription(V2aBrakeStaFb, '/pix_hooke/v2a_brakestafb', self.brake_topic_callback, 10)
        self.create_subscription(V2aDriveStaFb, '/pix_hooke/v2a_drivestafb', self.drive_topic_callback, 10)
        self.create_subscription(V2aSteerStaFb, '/pix_hooke/v2a_steerstafb', self.steer_topic_callback, 10)
        self.create_subscription(Imu, '/gnss/chc/imu', self.imu_topic_callback, 10)
        #self.create_subscription(Frame, '/from_can_bus', self.can_topic_callback, 10)
        

    def pitch_topic_callback(self, msg):
        #self.get_logger().info('Pitch angle in degrees is: "%f"' % msg.data)
        self.pitch_angle = msg.data


    def brake_topic_callback(self, msg):
          #self.get_logger().info('Brake intensity is: ')
          self.braking = msg.vcu_chassis_brake_padl_fb

    def drive_topic_callback(self, msg):
          #self.get_logger().info('Throttle intensity is: ')
          self.throttling = msg.vcu_chassis_throttle_padl_fb

          #self.get_logger().info('Velocity is: ')
          #print(msg.vcu_chassis_speed_fb)
          self.velocity = msg.vcu_chassis_speed_fb

    def steer_topic_callback(self, msg):
          #self.get_logger().info('Steering intensity is: ')
          #print(msg.vcu_chassis_steer_angle_fb)
          self.steering = msg.vcu_chassis_steer_angle_fb

    def imu_topic_callback(self, msg):
        #self.get_logger().info('Acceleration is: ')
        self.acceleration = msg.linear_acceleration.x


    def test_callback(self):
          
          
          # THROTTLING SCENARIO to train throttling model

          if(self.braking == 0 and self.throttling >= self.THROTTLE_DEADZONE and abs(self.steering) < self.STEERING_THRESHOLD):
                
                #low velocity scenario

                if(0 < abs(self.velocity) <= self.SPEED_THRESHOLD):
                      
                      if(self.throttling <= 25 and self.i < self.MAX_DATA):
                            self.vel.append(self.velocity)
                            self.cmd.append(self.throttling)
                            self.acc.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.i += 1
                            self.progress_bar1.update(1)



                      elif(25 < self.throttling <= 35 and self.j < self.MAX_DATA):
                            self.vel.append(self.velocity)
                            self.cmd.append(self.throttling)
                            self.acc.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.j += 1
                            self.progress_bar2.update(1)


                      elif(self.throttling > 35 and self.h < self.MAX_DATA):
                            self.vel.append(self.velocity)
                            self.cmd.append(self.throttling)
                            self.acc.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.h += 1
                            self.progress_bar3.update(1)


                #high velocity scenario

                elif(abs(self.velocity) > self.SPEED_THRESHOLD):
                      
                      if(self.throttling <= 25 and self.a < self.MAX_DATA):
                            self.vel.append(self.velocity)
                            self.cmd.append(self.throttling)
                            self.acc.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.a += 1
                            self.progress_bar4.update(1)

                      elif(25 < self.throttling <= 35 and self.b < self.MAX_DATA):
                            self.vel.append(self.velocity)
                            self.cmd.append(self.throttling)
                            self.acc.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.b += 1
                            self.progress_bar5.update(1)

                      elif(self.throttling > 35 and self.c < self.MAX_DATA):
                            self.vel.append(self.velocity)
                            self.cmd.append(self.throttling)
                            self.acc.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.c += 1
                            self.progress_bar6.update(1)


          if (self.i == self.MAX_DATA and self.j == self.MAX_DATA and self.h == self.MAX_DATA and self.a == self.MAX_DATA and self.b == self.MAX_DATA and self.c == self.MAX_DATA):
               dict = {'Velocity': self.vel, 'Throttling': self.cmd, 'Acceleration': self.acc}
               df = pd.DataFrame(dict)
               df.to_csv('throttling.csv') 

               self.i = self.MAX_DATA + 1



          # BRAKING SCENARIO to train braking model

          if(self.braking >= self.BRAKE_DEADZONE and self.throttling == 0 and abs(self.steering) < self.STEERING_THRESHOLD):
                
                #low velocity scenario

                if(0 < abs(self.velocity) <= self.SPEED_THRESHOLD):
                      
                      if(self.braking <= 20 and self.ii < self.MAX_DATA):
                            self.velb.append(self.velocity)
                            self.cmdb.append(self.braking)
                            self.accb.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.ii += 1
                            self.progress_bar7.update(1)

                      elif(20 < self.braking <= 30 and self.jj < self.MAX_DATA):
                            self.velb.append(self.velocity)
                            self.cmdb.append(self.braking)
                            self.accb.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.jj += 1
                            self.progress_bar8.update(1)

                      elif(self.braking > 30 and self.hh < self.MAX_DATA):
                            self.velb.append(self.velocity)
                            self.cmdb.append(self.braking)
                            self.accb.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.hh += 1
                            self.progress_bar9.update(1)


                #high velocity scenario

                elif(abs(self.velocity) > self.SPEED_THRESHOLD):
                      
                      if(self.braking <= 20 and self.aa < self.MAX_DATA):
                            self.velb.append(self.velocity)
                            self.cmdb.append(self.braking)
                            self.accb.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.aa += 1
                            self.progress_bar10.update(1)

                      elif(20 < self.braking <= 30 and self.bb < self.MAX_DATA):
                            self.velb.append(self.velocity)
                            self.cmdb.append(self.braking)
                            self.accb.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.b += 1
                            self.progress_bar11.update(1)

                      elif(self.braking > 30 and self.cc < self.MAX_DATA):
                            self.velb.append(self.velocity)
                            self.cmdb.append(self.braking)
                            self.accb.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                            self.cc += 1   
                            self.progress_bar12.update(1)         


          if (self.ii == self.MAX_DATA and self.jj == self.MAX_DATA and self.hh == self.MAX_DATA and self.aa == self.MAX_DATA and self.bb == self.MAX_DATA and self.cc == self.MAX_DATA):
               dict1 = {'Velocity': self.velb, 'Braking': self.cmdb, 'Acceleration': self.accb}
               df = pd.DataFrame(dict1)
               df.to_csv('braking.csv') 

               self.ii = self.MAX_DATA + 1


                           
                            
                            

                         
                            

def main():
        rclpy.init()
        node = primotest()
        rclpy.spin(node)

if __name__ == '__main__':
        main()