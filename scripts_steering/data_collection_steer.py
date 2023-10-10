#! /usr/bin/env python3

import rclpy
import rclpy.node
import pandas as pd
import math
from collections import deque
from statistics import mean

from tier4_vehicle_msgs.msg import ActuationCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from tqdm import tqdm

class primotest(rclpy.node.Node):

      def __init__(self):

            super().__init__('primo_test')

            self.i = 0
            self.j = 0
            self.h = 0
            self.k = 0
            self.a = 0
            self.ii = 0
            self.jj = 0
            self.hh = 0
            self.kk = 0
            self.aa = 0
            self.vel = []
            self.cmd = []
            self.acc = []
            self.acc2 = []
            self.pitch = []
            self.steer = []
            
            
            # data sharing member variables
            
            self.throttling = 0.0
            self.throttling_prec = 0.0
            self.steering = 0.0
            self.acceleration = 0.0
            self.velocity = 0.0
            self.pitch_angle = 0.0


            self.MAX_DATA = 3000
            self.NUM_OF_QUEUE = 20
            self.MAX_VELOCITY = 5.0/3.6
            self.THROTTLE_THRESHOLD = 10
            self.CONSISTENCY_TRESHOLD = 20   
            self.STEERING_THR1 = 0.2
            self.STEERING_THR2 = 0.4
            self.STEERING_THR3 = 0.6
            self.STEERING_THR4 = 0.8         
            

            self.g = 9.80665

            self.progress_bar0 = tqdm(total = self.MAX_DATA, desc = " Low throttle | Steering: 0.0 - " + str(self.STEERING_THR1))
            self.progress_bar1 = tqdm(total = self.MAX_DATA, desc = " Low throttle | Steering: " + str(self.STEERING_THR1) + " - " + str(self.STEERING_THR2))
            self.progress_bar2 = tqdm(total = self.MAX_DATA, desc = " Low throttle | Steering: " + str(self.STEERING_THR2) + " - " + str(self.STEERING_THR3))
            self.progress_bar3 = tqdm(total = self.MAX_DATA, desc = " Low throttle | Steering: " + str(self.STEERING_THR3) + " - " + str(self.STEERING_THR4))
            self.progress_bar4 = tqdm(total = self.MAX_DATA, desc = " Low throttle | Steering: " + str(self.STEERING_THR4) + " - 1.0")
            self.progress_bar5 = tqdm(total = self.MAX_DATA, desc = "High throttle | Steering: 0.0 - " + str(self.STEERING_THR1))
            self.progress_bar6 = tqdm(total = self.MAX_DATA, desc = "High throttle | Steering: " + str(self.STEERING_THR1) + " - " + str(self.STEERING_THR2))
            self.progress_bar7 = tqdm(total = self.MAX_DATA, desc = "High throttle | Steering: " + str(self.STEERING_THR2) + " - " + str(self.STEERING_THR3))
            self.progress_bar8 = tqdm(total = self.MAX_DATA, desc = "High throttle | Steering: " + str(self.STEERING_THR3) + " - " + str(self.STEERING_THR4))
            self.progress_bar9 = tqdm(total = self.MAX_DATA, desc = "High throttle | Steering: " + str(self.STEERING_THR4) + " - 1.0")

            
            self.create_subscription(Float32, '/sensing/gnss/chc/pitch', self.pitch_topic_callback, 1)
            self.create_subscription(ActuationCommand, '/actuation_input', self.drive_topic_callback, 1)
            self.create_subscription(ActuationCommand, '/actuation_input', self.steer_topic_callback, 1)
            self.create_subscription(VelocityReport, '/velocity_input', self.velocity_topic_callback, 1)
            self.create_subscription(Imu, '/sensing/gnss/chc/imu', self.imu_topic_callback, 1)
            self.timer = self.create_timer(0.02, self.test_callback)

            # make sure to record these data: ros2 bag record /sensing/gnss/chc/pitch /actuation_input /velocity_input /sensing/gnss/chc/imu
            
            self.queue_velocity = deque()
            self.queue_acceleration = deque()
            self.queue_acceleration_mov_avg = deque()
            self.queue_pitch_angle = deque()
            self.queue_pitch_angle_mov_avg = deque()
            self.queue_throttle = deque()
            self.queue_steering = deque()
            
            

            


      def pitch_topic_callback(self, msg):
            
            self.pitch_angle = float(msg.data)
            # apply a mean filter
            if(len(self.queue_pitch_angle)<self.NUM_OF_QUEUE):
                  self.queue_pitch_angle.append(self.pitch_angle)
            else:
                  self.queue_pitch_angle.popleft()
                  
            
                        
                        
                  
      def velocity_topic_callback(self, msg):
            self.velocity = float(msg.longitudinal_velocity)
            if(len(self.queue_velocity)<self.NUM_OF_QUEUE):
                  self.queue_velocity.append(self.velocity)
            else:
                  self.queue_velocity.popleft()


                  

      def drive_topic_callback(self, msg):
            
            self.throttling = float(msg.accel_cmd)
            if(len(self.queue_throttle)<self.NUM_OF_QUEUE):
                  self.queue_throttle.append(self.throttling)
            else:
                  self.queue_throttle.popleft()
                  
                  

      def steer_topic_callback(self, msg):
            
            self.steering = float(msg.steer_cmd)
            if(len(self.queue_steering)<self.NUM_OF_QUEUE):
                  self.queue_steering.append(self.steering)
            else:
                  self.queue_steering.popleft()
               
               
                    

      def imu_topic_callback(self, msg):
            
            self.acceleration = float(msg.linear_acceleration.x)
            if(len(self.queue_acceleration)<self.NUM_OF_QUEUE):
                  self.queue_acceleration.append(self.acceleration)
            else:
                  self.queue_acceleration.popleft()
                  

                        
                        
                        
      def collection_steering(self):
            
            self.vel.append(abs(mean(self.queue_velocity)))
            self.cmd.append(mean(self.queue_throttle))
            self.steer.append(abs(mean(self.queue_steering)))
            if(mean(self.queue_velocity) < 0):
                  self.acc.append(-1*mean(self.queue_acceleration)-self.g*math.sin(math.radians(mean(self.queue_pitch_angle))))
                  self.acc2.append(-1*mean(self.queue_acceleration))
                  self.pitch.append(-1*mean(self.queue_pitch_angle))
            else:
                  self.acc.append(mean(self.queue_acceleration)-self.g*math.sin(math.radians(mean(self.queue_pitch_angle))))
                  self.acc2.append(mean(self.queue_acceleration))
                  self.pitch.append(mean(self.queue_pitch_angle))
                              
                              
            # save data in csv file                 
            dict1 = {'Velocity': self.vel, 'Throttling': self.cmd, 'Steering': self.steer, 'Acceleration_with_pitch_comp': self.acc, 'Acceleration_measured': self.acc2, 'Pitch_angle': self.pitch, 'k': self.k, 'i': self.i, 'j': self.j, 'h': self.h, 'd': self.d, 'a': self.a, 'b': self.b, 'c': self.c}
            df1 = pd.DataFrame(dict1)
            df1.to_csv('steering.csv') 

            self.throttling_prec = mean(self.queue_throttle)
            
            
            
            
     
            
            
            
            
            
                  
                        
                  
      


      def test_callback(self):
            
            
            

            if(0 < abs(self.velocity) <= self.MAX_VELOCITY and abs(self.throttling_prec-mean(self.queue_throttle)) <= self.CONSISTENCY_TRESHOLD):
                  
                  #low velocity scenario

                  if(self.throttling <= self.THROTTLE_THRESHOLD):
                        
                        if(0 < abs(self.steering) <= self.STEERING_THR1 and self.k < self.MAX_DATA):
                              
                              self.collection_steering()
                              
                              self.progress_bar0.update(1)
                              
                              self.k += 1

                        
                        elif(self.STEERING_THR1 < abs(self.steering) <= self.STEERING_THR2 and self.i < self.MAX_DATA):
                              
                              self.collection_steering()
                              
                              self.progress_bar1.update(1)
                              
                              self.i += 1


                        elif(self.STEERING_THR2 < abs(self.steering) <= self.STEERING_THR3 and self.j < self.MAX_DATA):
                              
                              self.collection_steering()
                              
                              self.progress_bar2.update(1)
                             
                              self.j += 1


                        elif(self.STEERING_THR3 < abs(self.throttling) <= self.STEERING_THR4 and self.h < self.MAX_DATA):
                              
                              self.collection_steering()
                              
                              self.progress_bar3.update(1)
                              
                              self.h += 1


                        elif(abs(self.steering) > self.STEERING_THR4 and self.a < self.MAX_DATA):

                              self.collection_steering()

                              self.progress_bar4.update(1)

                              self.a += 1


                  #high velocity scenario

                  elif(self.throttling > self.THROTTLE_THRESHOLD):
                        
                        if(0 < abs(self.steering) <= self.STEERING_THR1 and self.kk < self.MAX_DATA):
                              
                              self.collection_steering()
                              
                              self.progress_bar5.update(1)
                              
                              self.kk += 1

                        
                        elif(self.STEERING_THR1 < abs(self.steering) <= self.STEERING_THR2 and self.ii < self.MAX_DATA):
                              
                              self.collection_steering()
                              
                              self.progress_bar6.update(1)
                              
                              self.ii += 1


                        elif(self.STEERING_THR2 < abs(self.steering) <= self.STEERING_THR3 and self.jj < self.MAX_DATA):
                              
                              self.collection_steering()
                              
                              self.progress_bar7.update(1)
                             
                              self.jj += 1


                        elif(self.STEERING_THR3 < abs(self.throttling) <= self.STEERING_THR4 and self.hh < self.MAX_DATA):
                              
                              self.collection_steering()
                              
                              self.progress_bar8.update(1)
                              
                              self.hh += 1


                        elif(abs(self.steering) > self.STEERING_THR4 and self.aa < self.MAX_DATA):

                              self.collection_steering()

                              self.progress_bar9.update(1)

                              self.aa += 1


                              
                              
                              

                                                    

def main():
      rclpy.init()
      node = primotest()
      rclpy.spin(node)

if __name__ == '__main__':
      main()