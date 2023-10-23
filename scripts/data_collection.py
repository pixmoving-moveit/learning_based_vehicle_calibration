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
            self.b = 0
            self.c = 0
            self.d = 0
            self.ii = 0
            self.jj = 0
            self.hh = 0
            self.kk = 0
            self.aa = 0
            self.bb = 0
            self.cc = 0
            self.dd = 0
            self.vel = []
            self.cmd = []
            self.acc = []
            self.acc2 = []
            self.velb = []
            self.cmdb = []
            self.accb = []
            self.accb2 = []
            self.pitch = []
            self.pitch2 = []
            self.flag = 0
            
            
            # data sharing member variables
            self.braking = 0.0
            self.braking_prec = 0.0
            self.throttling = 0.0
            self.throttling_prec = 0.0
            self.steering = 0.0
            self.acceleration = 0.0
            self.velocity = 0.0
            self.pitch_angle = 0.0


            # here you can tune the parameters according to your needs

            self.MAX_DATA = 3000
            self.NUM_OF_QUEUE = 20
            self.SPEED_THRESHOLD = 10.0/3.6
            self.STEERING_THRESHOLD = 2/25
            self.THROTTLE_DEADZONE = 5
            self.BRAKE_DEADZONE = 5
            self.MAX_VELOCITY = 40.0/3.6
            self.THROTTLE_THRESHOLD1 = 30
            self.THROTTLE_THRESHOLD2 = 55
            self.BRAKE_THRESHOLD1 = 15
            self.BRAKE_THRESHOLD2 = 25
            self.CONSISTENCY_TRESHOLD = 20            
            
            self.g = 9.80665

            self.progress_bar0 = tqdm(total = self.MAX_DATA, desc = "Low speed: 0 - Throttle deadzone  ")
            self.progress_bar1 = tqdm(total = self.MAX_DATA, desc = "Low speed: Throttle deadzone - " + str(self.THROTTLE_THRESHOLD1) + " ")
            self.progress_bar2 = tqdm(total = self.MAX_DATA, desc = "Low speed: Throttle " + str(self.THROTTLE_THRESHOLD1) + " - " + str(self.THROTTLE_THRESHOLD2) + "       ")
            self.progress_bar3 = tqdm(total = self.MAX_DATA, desc = "Low speed: Throttle > " + str(self.THROTTLE_THRESHOLD2) + "          ")
            self.progress_bar4 = tqdm(total = self.MAX_DATA, desc = "High speed: 0 - Throttle deadzone ")
            self.progress_bar5 = tqdm(total = self.MAX_DATA, desc = "High speed: Throttle deadzone - " + str(self.THROTTLE_THRESHOLD2))
            self.progress_bar6 = tqdm(total = self.MAX_DATA, desc = "High speed: Throttle " + str(self.THROTTLE_THRESHOLD1) + " - " + str(self.THROTTLE_THRESHOLD2) + "      ")
            self.progress_bar7 = tqdm(total = self.MAX_DATA, desc = "High speed: Throttle > " + str(self.THROTTLE_THRESHOLD2) + "         ")
            self.progress_bar8 = tqdm(total = self.MAX_DATA, desc = "Low speed: 0 - Brake deadzone     ")
            self.progress_bar9 = tqdm(total = self.MAX_DATA, desc = "Low speed: Brake deadzone - " + str(self.BRAKE_THRESHOLD1) + "    ")
            self.progress_bar10 = tqdm(total = self.MAX_DATA, desc = "Low speed: Brake " + str(self.BRAKE_THRESHOLD1) + " - " + str(self.BRAKE_THRESHOLD2) + "          ") 
            self.progress_bar11 = tqdm(total = self.MAX_DATA, desc = "Low speed: Brake > " + str(self.BRAKE_THRESHOLD2) + "             ")
            self.progress_bar12 = tqdm(total = self.MAX_DATA, desc = "High speed: 0 - Brake deadzone    ")
            self.progress_bar13 = tqdm(total = self.MAX_DATA, desc = "High speed: Brake deadzone - " + str(self.BRAKE_THRESHOLD1) + "   ")
            self.progress_bar14 = tqdm(total = self.MAX_DATA, desc = "High speed: Brake " + str(self.BRAKE_THRESHOLD1) + " - " + str(self.BRAKE_THRESHOLD2) + "         ")
            self.progress_bar15 = tqdm(total = self.MAX_DATA, desc = "High speed: Brake > " + str(self.BRAKE_THRESHOLD2) + "            ")

            
            self.create_subscription(Float32, '/sensing/gnss/chc/pitch', self.pitch_topic_callback, 1)
            self.create_subscription(ActuationCommand, '/actuation_input', self.brake_topic_callback, 1)
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
            self.queue_braking = deque()
            
            

            


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


      def brake_topic_callback(self, msg):
            
            self.braking = float(msg.brake_cmd)
            if(len(self.queue_braking)<self.NUM_OF_QUEUE):
                  self.queue_braking.append(self.braking)
            else:
                  self.queue_braking.popleft()
                  

      def drive_topic_callback(self, msg):
            
            self.throttling = float(msg.accel_cmd)
            if(len(self.queue_throttle)<self.NUM_OF_QUEUE):
                  self.queue_throttle.append(self.throttling)
            else:
                  self.queue_throttle.popleft()
                  
                  

      def steer_topic_callback(self, msg):
            
            self.steering = float(msg.steer_cmd)
            
            

      def imu_topic_callback(self, msg):
            
            self.acceleration = float(msg.linear_acceleration.x)
            if(len(self.queue_acceleration)<self.NUM_OF_QUEUE):
                  self.queue_acceleration.append(self.acceleration)
            else:
                  self.queue_acceleration.popleft()
                        
                        
                        
      def collection_throttling(self):
            
            self.vel.append(abs(mean(self.queue_velocity)))
            self.cmd.append(mean(self.queue_throttle))
            if(mean(self.queue_velocity) < 0):
                  self.acc.append(-1*mean(self.queue_acceleration)-self.g*math.sin(math.radians(-1*mean(self.queue_pitch_angle))))
                  self.acc2.append(-1*mean(self.queue_acceleration))
                  self.pitch.append(-1*mean(self.queue_pitch_angle))
            else:
                  self.acc.append(mean(self.queue_acceleration)-self.g*math.sin(math.radians(mean(self.queue_pitch_angle))))
                  self.acc2.append(mean(self.queue_acceleration))
                  self.pitch.append(mean(self.queue_pitch_angle))
                              
                              
            # save data in csv file                 
            dict1 = {'Velocity': self.vel, 'Throttling': self.cmd, 'Acceleration_with_pitch_comp': self.acc, 'Acceleration_measured': self.acc2, 'Pitch_angle': self.pitch, 'k': self.k, 'i': self.i, 'j': self.j, 'h': self.h, 'd': self.d, 'a': self.a, 'b': self.b, 'c': self.c}
            df1 = pd.DataFrame(dict1)
            df1.to_csv('throttling.csv') 

            self.throttling_prec = mean(self.queue_throttle)
            
            
            
            
      def collection_braking(self):
            
            self.velb.append(abs(mean(self.queue_velocity)))
            self.cmdb.append(mean(self.queue_braking))
            if(mean(self.queue_velocity) < 0):
                  self.accb.append(-1*mean(self.queue_acceleration)-self.g*math.sin(math.radians(mean(self.queue_pitch_angle))))
                  self.accb2.append(-1*mean(self.queue_acceleration))
                  self.pitch2.append(-1*mean(self.queue_pitch_angle))
            else:
                  self.accb.append(mean(self.queue_acceleration)-self.g*math.sin(math.radians(mean(self.queue_pitch_angle))))
                  self.accb2.append(mean(self.queue_acceleration))
                  self.pitch2.append(mean(self.queue_pitch_angle))
                              
                              
                              
            dict2 = {'Velocity': self.velb, 'Braking': self.cmdb, 'Acceleration_with_pitch_comp': self.accb, 'Acceleration_measured': self.accb2, 'Pitch_angle': self.pitch2, 'kk': self.kk, 'ii': self.ii, 'jj': self.jj, 'hh': self.hh, 'dd': self.dd, 'aa': self.aa, 'bb': self.bb, 'cc': self.cc}
            df2 = pd.DataFrame(dict2)
            df2.to_csv('braking.csv') 

            self.braking_prec = mean(self.queue_braking)
            
            
            
            
            
                  
                        
                  
      


      def test_callback(self):
            
            
            # THROTTLING SCENARIO to train throttling model

            if(self.braking == 0 and abs(self.steering) < self.STEERING_THRESHOLD and abs(self.throttling_prec-mean(self.queue_throttle)) <= self.CONSISTENCY_TRESHOLD):
                  
                  #low velocity scenario

                  if(0 < abs(self.velocity) <= self.SPEED_THRESHOLD):
                        
                        if(0 <= self.throttling <= self.THROTTLE_DEADZONE and self.k < self.MAX_DATA and self.flag == 0):
                              
                              self.collection_throttling()
                              
                              self.progress_bar0.update(1)
                              
                              self.k += 1

                        
                        elif(self.THROTTLE_DEADZONE < self.throttling <= self.THROTTLE_THRESHOLD1 and self.i < self.MAX_DATA):
                              
                              self.collection_throttling()
                              
                              self.progress_bar1.update(1)
                              self.flag = 0
                              
                              self.i += 1


                        elif(self.THROTTLE_THRESHOLD1 < self.throttling <= self.THROTTLE_THRESHOLD2 and self.j < self.MAX_DATA):
                              
                              self.collection_throttling()
                              
                              self.progress_bar2.update(1)
                              self.flag = 0
                             
                              self.j += 1


                        elif(self.throttling > self.THROTTLE_THRESHOLD2 and self.h < self.MAX_DATA):
                              
                              self.collection_throttling()
                              
                              self.progress_bar3.update(1)
                              self.flag = 0
                              
                              self.h += 1


                  #high velocity scenario

                  elif(self.SPEED_THRESHOLD < abs(self.velocity) <= self.MAX_VELOCITY):
                        
                        if(0 <= self.throttling <= self.THROTTLE_DEADZONE and self.d < self.MAX_DATA and self.flag == 0):
                              
                              self.collection_throttling()
                              
                              self.progress_bar4.update(1)
                              
                              self.d += 1


                        elif(self.THROTTLE_DEADZONE < self.throttling <= self.THROTTLE_THRESHOLD1 and self.a < self.MAX_DATA):
                              
                              self.collection_throttling()
                              
                              self.progress_bar5.update(1)
                              self.flag = 0
                              
                              self.a += 1


                        elif(self.THROTTLE_THRESHOLD1 < self.throttling <= self.THROTTLE_THRESHOLD2 and self.b < self.MAX_DATA):
                              
                              self.collection_throttling()
                              
                              self.progress_bar6.update(1)
                              self.flag = 0
                           
                              self.b += 1

                        elif(self.throttling > self.THROTTLE_THRESHOLD2 and self.c < self.MAX_DATA):
                              
                              self.collection_throttling()
                              
                              self.progress_bar7.update(1)
                              self.flag = 0
                              
                              self.c += 1


            
            
            



            # BRAKING SCENARIO to train braking model

            if(self.throttling == 0 and abs(self.steering) < self.STEERING_THRESHOLD and abs(self.braking_prec-mean(self.queue_braking)) <= self.CONSISTENCY_TRESHOLD):
                  
                  #low velocity scenario

                  if(0 < abs(self.velocity) <= self.SPEED_THRESHOLD):
                        
                        if(0 <= self.braking <= self.BRAKE_DEADZONE and self.kk < self.MAX_DATA and self.flag == 1):
                              
                              self.collection_braking()
                              
                              self.progress_bar8.update(1)
                              
                              self.kk += 1


                        elif(self.BRAKE_DEADZONE < self.braking <= self.BRAKE_THRESHOLD1 and self.ii < self.MAX_DATA):
                              
                              self.collection_braking()
                              
                              self.progress_bar9.update(1)
                              self.flag = 1
                              
                              self.ii += 1


                        elif(self.BRAKE_THRESHOLD1 < self.braking <= self.BRAKE_THRESHOLD2 and self.jj < self.MAX_DATA):
                              
                              self.collection_braking()
                              
                              self.progress_bar10.update(1)
                              self.flag = 1
                              
                              self.jj += 1


                        elif(self.braking > self.BRAKE_THRESHOLD2 and self.hh < self.MAX_DATA):
                              
                              self.collection_braking()
                              
                              self.progress_bar11.update(1)
                              self.flag = 1
                              
                              self.hh += 1


                  #high velocity scenario

                  elif(self.SPEED_THRESHOLD < abs(self.velocity) <= self.MAX_VELOCITY):
                        
                        if(0 <= self.braking <= self.BRAKE_DEADZONE and self.dd < self.MAX_DATA and self.flag == 1):
                              
                              self.collection_braking()
                              
                              self.progress_bar12.update(1)
                              
                              self.dd += 1

                        elif(self.BRAKE_DEADZONE < self.braking <= self.BRAKE_THRESHOLD1 and self.aa < self.MAX_DATA):
                              
                              self.collection_braking()
                              
                              self.progress_bar13.update(1)
                              self.flag = 1
                             
                              self.aa += 1


                        elif(self.BRAKE_THRESHOLD1 < self.braking <= self.BRAKE_THRESHOLD2 and self.bb < self.MAX_DATA):
                              
                              self.collection_braking()
                              
                              self.progress_bar14.update(1)
                              self.flag = 1
                              
                              self.bb += 1
                              

                        elif(self.braking > self.BRAKE_THRESHOLD2 and self.cc < self.MAX_DATA):
                              
                              self.collection_braking()
                              
                              self.progress_bar15.update(1)
                              self.flag = 1  
                              
                              self.cc += 1  


            
            

                              
                              
                              

                                                    

def main():
      rclpy.init()
      node = primotest()
      rclpy.spin(node)

if __name__ == '__main__':
      main()