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
            self.vel1 = []
            self.cmd1 = []
            self.acc1 = []
            self.accp1 = []
            self.pitch1 = []
            self.steer1 = []
            self.vel2 = []
            self.cmd2 = []
            self.acc2 = []
            self.accp2 = []
            self.pitch2 = []
            self.steer2 = []
            self.vel3 = []
            self.cmd3 = []
            self.acc3 = []
            self.accp3 = []
            self.pitch3 = []
            self.steer3 = []
            self.vel4 = []
            self.cmd4 = []
            self.acc4 = []
            self.accp4 = []
            self.pitch4 = []
            self.steer4 = []
            self.vel5 = []
            self.cmd5 = []
            self.acc5 = []
            self.accp5 = []
            self.pitch5 = []
            self.steer5 = []
            
            
            # data sharing member variables
            
            self.throttling = 0.0
            self.throttling_prec = 0.0
            self.steering = 0.0
            self.acceleration = 0.0
            self.velocity = 0.0
            self.pitch_angle = 0.0


            # here you can tune these parameters according to your needs

            self.MAX_DATA = 10000
            self.MAX_VELOCITY = 7.0/3.6
            self.THROTTLE_THRESHOLD = 12
            self.CONSISTENCY_TRESHOLD = 20   
            self.STEERING_THR1 = 0.05
            self.STEERING_THR2 = 0.20
            self.STEERING_THR3 = 0.40
            self.STEERING_THR4 = 0.60
            self.STEERING_THR5 = 0.80 

            

            self.g = 9.80665

            self.progress_bar0 = tqdm(total = self.MAX_DATA, desc = " Low throttle | Steering: " + str(self.STEERING_THR1) + " - " + str(self.STEERING_THR2))
            self.progress_bar1 = tqdm(total = self.MAX_DATA, desc = " Low throttle | Steering:  " + str(self.STEERING_THR2) + " - " + str(self.STEERING_THR3))
            self.progress_bar2 = tqdm(total = self.MAX_DATA, desc = " Low throttle | Steering:  " + str(self.STEERING_THR3) + " - " + str(self.STEERING_THR4))
            self.progress_bar3 = tqdm(total = self.MAX_DATA, desc = " Low throttle | Steering:  " + str(self.STEERING_THR4) + " - " + str(self.STEERING_THR5))
            self.progress_bar4 = tqdm(total = self.MAX_DATA, desc = " Low throttle | Steering:  " + str(self.STEERING_THR5) + " - 1.0")
            self.progress_bar10 = tqdm(total = self.MAX_DATA, desc = "High throttle | Steering: " + str(self.STEERING_THR1) + " - " + str(self.STEERING_THR2))
            self.progress_bar11 = tqdm(total = self.MAX_DATA, desc = "High throttle | Steering:  " + str(self.STEERING_THR2) + " - " + str(self.STEERING_THR3))
            self.progress_bar12 = tqdm(total = self.MAX_DATA, desc = "High throttle | Steering:  " + str(self.STEERING_THR3) + " - " + str(self.STEERING_THR4))
            self.progress_bar13 = tqdm(total = self.MAX_DATA, desc = "High throttle | Steering:  " + str(self.STEERING_THR4) + " - " + str(self.STEERING_THR5))
            self.progress_bar14 = tqdm(total = self.MAX_DATA, desc = "High throttle | Steering:  " + str(self.STEERING_THR5) + " - 1.0")

            
            self.create_subscription(Float32, '/sensing/gnss/chc/pitch', self.pitch_topic_callback, 1)
            self.create_subscription(ActuationCommand, '/actuation_input', self.drive_topic_callback, 1)
            self.create_subscription(ActuationCommand, '/actuation_input', self.steer_topic_callback, 1)
            self.create_subscription(VelocityReport, '/velocity_input', self.velocity_topic_callback, 1)
            self.create_subscription(Imu, '/sensing/gnss/chc/imu', self.imu_topic_callback, 1)
            self.timer = self.create_timer(0.02, self.test_callback)

            # make sure to record these data: ros2 bag record /sensing/gnss/chc/pitch /actuation_input /velocity_input /sensing/gnss/chc/imu
            
        
            
            

            
      # WE WILL FILTER THE DATA DURING THE POST-PROCESSING, SO IN THIS CASE WE DON'T USE DE QUEUE

      def pitch_topic_callback(self, msg):
            
            self.pitch_angle = float(msg.data)
            
                          
                  
      def velocity_topic_callback(self, msg):
            self.velocity = float(msg.longitudinal_velocity)
            


      def drive_topic_callback(self, msg):
            
            self.throttling = float(msg.accel_cmd)
                  
                  

      def steer_topic_callback(self, msg):
            
            self.steering = float(msg.steer_cmd)
                   
                    

      def imu_topic_callback(self, msg):
            
            self.acceleration = float(msg.linear_acceleration.x)
        
                  
                        
                        
                        
      def collection_steering1(self):
            
            self.vel1.append(abs(self.velocity))
            self.cmd1.append(self.throttling)
            self.steer1.append(abs(self.steering))
            if(self.velocity < 0):
                  self.acc1.append(-1*self.acceleration-self.g*math.sin(math.radians(-1*self.pitch_angle)))
                  self.accp1.append(-1*self.acceleration)
                  self.pitch1.append(-1*self.pitch_angle)
            else:
                  self.acc1.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                  self.accp1.append(self.acceleration)
                  self.pitch1.append(self.pitch_angle)
                              
                              
            # save data in csv file                 
            dict1 = {'Velocity': self.vel1, 'Throttling': self.cmd1, 'Steering': self.steer1, 'Acceleration_with_pitch_comp': self.acc1, 'Acceleration_measured': self.accp1, 'Pitch_angle': self.pitch1}
            df1 = pd.DataFrame(dict1)
            df1.to_csv('steering_01.csv') 

            #self.throttling_prec = self.throttling



      def collection_steering2(self):
            
            self.vel2.append(abs(self.velocity))
            self.cmd2.append(self.throttling)
            self.steer2.append(abs(self.steering))
            if(self.velocity < 0):
                  self.acc2.append(-1*self.acceleration-self.g*math.sin(math.radians(-1*self.pitch_angle)))
                  self.accp2.append(-1*self.acceleration)
                  self.pitch2.append(-1*self.pitch_angle)
            else:
                  self.acc2.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                  self.accp2.append(self.acceleration)
                  self.pitch2.append(self.pitch_angle)
                              
                              
            # save data in csv file                 
            dict2 = {'Velocity': self.vel2, 'Throttling': self.cmd2, 'Steering': self.steer2, 'Acceleration_with_pitch_comp': self.acc2, 'Acceleration_measured': self.accp2, 'Pitch_angle': self.pitch2}
            df2 = pd.DataFrame(dict2)
            df2.to_csv('steering_02.csv') 

            #self.throttling_prec = self.throttling     



      def collection_steering3(self):
            
            self.vel3.append(abs(self.velocity))
            self.cmd3.append(self.throttling)
            self.steer3.append(abs(self.steering))
            if(self.velocity < 0):
                  self.acc3.append(-1*self.acceleration-self.g*math.sin(math.radians(-1*self.pitch_angle)))
                  self.accp3.append(-1*self.acceleration)
                  self.pitch3.append(-1*self.pitch_angle)
            else:
                  self.acc3.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                  self.accp3.append(self.acceleration)
                  self.pitch3.append(self.pitch_angle)
                              
                              
            # save data in csv file                 
            dict3 = {'Velocity': self.vel3, 'Throttling': self.cmd3, 'Steering': self.steer3, 'Acceleration_with_pitch_comp': self.acc3, 'Acceleration_measured': self.accp3, 'Pitch_angle': self.pitch3}
            df3 = pd.DataFrame(dict3)
            df3.to_csv('steering_03.csv') 

            #self.throttling_prec = mean(self.queue_throttle)      



      def collection_steering4(self):
            
            self.vel4.append(abs(self.velocity))
            self.cmd4.append(self.throttling)
            self.steer4.append(abs(self.steering))
            if(self.velocity < 0):
                  self.acc4.append(-1*self.acceleration-self.g*math.sin(math.radians(-1*self.pitch_angle)))
                  self.accp4.append(-1*self.acceleration)
                  self.pitch4.append(-1*self.pitch_angle)
            else:
                  self.acc4.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                  self.accp4.append(self.acceleration)
                  self.pitch4.append(self.pitch_angle)
                              
                              
            # save data in csv file                 
            dict4 = {'Velocity': self.vel4, 'Throttling': self.cmd4, 'Steering': self.steer4, 'Acceleration_with_pitch_comp': self.acc4, 'Acceleration_measured': self.accp4, 'Pitch_angle': self.pitch4}
            df4 = pd.DataFrame(dict4)
            df4.to_csv('steering_04.csv') 

            #self.throttling_prec = mean(self.queue_throttle)  



      def collection_steering5(self):
            
            self.vel5.append(abs(self.velocity))
            self.cmd5.append(self.throttling)
            self.steer5.append(abs(self.steering))
            if(self.velocity < 0):
                  self.acc5.append(-1*self.acceleration-self.g*math.sin(math.radians(-1*self.pitch_angle)))
                  self.accp5.append(-1*self.acceleration)
                  self.pitch5.append(-1*self.pitch_angle)
            else:
                  self.acc5.append(self.acceleration-self.g*math.sin(math.radians(self.pitch_angle)))
                  self.accp5.append(self.acceleration)
                  self.pitch5.append(self.pitch_angle)
                              
                              
            # save data in csv file                 
            dict5 = {'Velocity': self.vel5, 'Throttling': self.cmd5, 'Steering': self.steer5, 'Acceleration_with_pitch_comp': self.acc5, 'Acceleration_measured': self.accp5, 'Pitch_angle': self.pitch5}
            df5 = pd.DataFrame(dict5)
            df5.to_csv('steering_05.csv') 

            #self.throttling_prec = mean(self.queue_throttle) 




      


      def test_callback(self):
            
            
            

            if(0 < abs(self.velocity) <= self.MAX_VELOCITY): # and abs(self.throttling_prec-mean(self.queue_throttle)) <= self.CONSISTENCY_TRESHOLD):
                  
                  #low throttling scenario

                  if(self.throttling <= self.THROTTLE_THRESHOLD):
                        
                        if(self.STEERING_THR1 <= abs(self.steering) < self.STEERING_THR2 and self.k < self.MAX_DATA):
                              
                              self.collection_steering1()
                              
                              self.progress_bar0.update(1)
                              
                              self.k += 1

                        
                        elif(self.STEERING_THR2 <= abs(self.steering) < self.STEERING_THR3 and self.i < self.MAX_DATA):
                              
                              self.collection_steering2()
                              
                              self.progress_bar1.update(1)
                              
                              self.i += 1


                        elif(self.STEERING_THR3 <= abs(self.steering) < self.STEERING_THR4 and self.j < self.MAX_DATA):
                              
                              self.collection_steering3()
                              
                              self.progress_bar2.update(1)
                             
                              self.j += 1


                        elif(self.STEERING_THR4 <= abs(self.steering) < self.STEERING_THR5 and self.h < self.MAX_DATA):
                              
                              self.collection_steering4()
                              
                              self.progress_bar3.update(1)
                              
                              self.h += 1


                        elif(abs(self.steering) >= self.STEERING_THR5 and self.a < self.MAX_DATA):

                              self.collection_steering5()

                              self.progress_bar4.update(1)

                              self.a += 1





                  #high throttling scenario

                  elif(self.throttling > self.THROTTLE_THRESHOLD):
                        
                        if(self.STEERING_THR1 <= abs(self.steering) < self.STEERING_THR2 and self.kk < self.MAX_DATA):
                              
                              self.collection_steering1()
                              
                              self.progress_bar10.update(1)
                              
                              self.kk += 1

                        
                        elif(self.STEERING_THR2 <= abs(self.steering) < self.STEERING_THR3 and self.ii < self.MAX_DATA):
                              
                              self.collection_steering2()
                              
                              self.progress_bar11.update(1)
                              
                              self.ii += 1


                        elif(self.STEERING_THR3 <= abs(self.steering) < self.STEERING_THR4 and self.jj < self.MAX_DATA):
                              
                              self.collection_steering3()
                              
                              self.progress_bar12.update(1)
                             
                              self.jj += 1


                        elif(self.STEERING_THR4 <= abs(self.steering) < self.STEERING_THR5 and self.hh < self.MAX_DATA):
                              
                              self.collection_steering4()
                              
                              self.progress_bar13.update(1)
                              
                              self.hh += 1


                        elif(abs(self.steering) >= self.STEERING_THR5 and self.aa < self.MAX_DATA):

                              self.collection_steering5()

                              self.progress_bar14.update(1)

                              self.aa += 1


                      
                              
                              

                                                    

def main():
      rclpy.init()
      node = primotest()
      rclpy.spin(node)

if __name__ == '__main__':
      main()