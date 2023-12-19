#! /usr/bin/env python3

import rclpy
import rclpy.node
import pandas as pd
import math
from collections import deque
from statistics import mean

from tier4_vehicle_msgs.msg import ActuationStatusStamped
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_auto_vehicle_msgs.msg import SteeringReport
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_msgs.msg import Header
import time
import rospy

from tqdm import tqdm

from learning_based_vehicle_calibration.msg import SteeringProgress
from learning_based_vehicle_calibration.msg import SteeringProcesses

class primotest(rclpy.node.Node):

      def __init__(self):

            super().__init__('primo_test')
            
            
            # data sharing member variables
            
            self.throttling = 0.0
            self.throttling_prec = 0.0
            self.steering = 0.0
            self.acceleration = 0.0
            self.velocity = 0.0
            self.pitch_angle = 0.0

            self.g = 9.80665



            # Load params from launch file

            self.MAX_DATA = self.declare_parameter('max_data').get_parameter_value().integer_value
            self.MAX_VELOCITY = self.declare_parameter('max_velocity').get_parameter_value().double_value
            self.THROTTLE_THRESHOLD = self.declare_parameter('throttle_threshold').get_parameter_value().integer_value
            self.STEERING_THR1 = self.declare_parameter('steering_threshold_1').get_parameter_value().double_value
            self.STEERING_THR2 = self.declare_parameter('steering_threshold_2').get_parameter_value().double_value
            self.STEERING_THR3 = self.declare_parameter('steering_threshold_3').get_parameter_value().double_value
            self.STEERING_THR4 = self.declare_parameter('steering_threshold_4').get_parameter_value().double_value
            self.STEERING_THR5 = self.declare_parameter('steering_threshold_5').get_parameter_value().double_value

            self.RECOVERY_MODE = self.declare_parameter('Recovery_Mode').get_parameter_value().bool_value


            if self.RECOVERY_MODE:
                  df_existing1 = pd.read_csv('steering_01.csv')
                  df_existing2 = pd.read_csv('steering_02.csv')
                  df_existing3 = pd.read_csv('steering_03.csv')
                  df_existing4 = pd.read_csv('steering_04.csv')
                  df_existing5 = pd.read_csv('steering_05.csv')

                  self.k = df_existing1['Index_low_cmd'].iloc[0]
                  self.kk = df_existing1['Index_high_cmd'].iloc[0]
                  self.vel1 = df_existing1['Velocity'].tolist()
                  self.cmd1 = df_existing1['Throttling'].tolist()
                  self.acc1 = df_existing1['Acceleration_with_pitch_comp'].tolist()
                  self.accp1 = df_existing1['Acceleration_measured'].tolist()
                  self.pitch1 = df_existing1['Pitch_angle'].tolist()
                  self.steer1 = df_existing1['Steering'].tolist()

                  self.i = df_existing2['Index_low_cmd'].iloc[0]
                  self.ii = df_existing2['Index_high_cmd'].iloc[0]
                  self.vel2 = df_existing2['Velocity'].tolist()
                  self.cmd2 = df_existing2['Throttling'].tolist()
                  self.acc2 = df_existing2['Acceleration_with_pitch_comp'].tolist()
                  self.accp2 = df_existing2['Acceleration_measured'].tolist()
                  self.pitch2 = df_existing2['Pitch_angle'].tolist()
                  self.steer2 = df_existing2['Steering'].tolist()

                  self.j = df_existing3['Index_low_cmd'].iloc[0]
                  self.jj = df_existing3['Index_high_cmd'].iloc[0]
                  self.vel3 = df_existing3['Velocity'].tolist()
                  self.cmd3 = df_existing3['Throttling'].tolist()
                  self.acc3 = df_existing3['Acceleration_with_pitch_comp'].tolist()
                  self.accp3 = df_existing3['Acceleration_measured'].tolist()
                  self.pitch3 = df_existing3['Pitch_angle'].tolist()
                  self.steer3 = df_existing3['Steering'].tolist()

                  self.h = df_existing4['Index_low_cmd'].iloc[0]
                  self.hh = df_existing4['Index_high_cmd'].iloc[0]
                  self.vel4 = df_existing4['Velocity'].tolist()
                  self.cmd4 = df_existing4['Throttling'].tolist()
                  self.acc4 = df_existing4['Acceleration_with_pitch_comp'].tolist()
                  self.accp4 = df_existing4['Acceleration_measured'].tolist()
                  self.pitch4 = df_existing4['Pitch_angle'].tolist()
                  self.steer4 = df_existing4['Steering'].tolist()

                  self.a = df_existing5['Index_low_cmd'].iloc[0]
                  self.aa = df_existing5['Index_high_cmd'].iloc[0]
                  self.vel5 = df_existing5['Velocity'].tolist()
                  self.cmd5 = df_existing5['Throttling'].tolist()
                  self.acc5 = df_existing5['Acceleration_with_pitch_comp'].tolist()
                  self.accp5 = df_existing5['Acceleration_measured'].tolist()
                  self.pitch5 = df_existing5['Pitch_angle'].tolist()
                  self.steer5 = df_existing5['Steering'].tolist()

            else:
                  self.i = self.j = self.h = self.k = self.a = self.ii = self.jj = self.hh = self.kk = self.aa = 0
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


            self.progress_bar0 = tqdm(total = self.MAX_DATA-self.k, desc = " Low throttle | Steering: " + str(self.STEERING_THR1) + " - " + str(self.STEERING_THR2), dynamic_ncols = True)
            self.progress_bar1 = tqdm(total = self.MAX_DATA-self.i, desc = " Low throttle | Steering:  " + str(self.STEERING_THR2) + " - " + str(self.STEERING_THR3), dynamic_ncols = True)
            self.progress_bar2 = tqdm(total = self.MAX_DATA-self.j, desc = " Low throttle | Steering:  " + str(self.STEERING_THR3) + " - " + str(self.STEERING_THR4), dynamic_ncols = True)
            self.progress_bar3 = tqdm(total = self.MAX_DATA-self.h, desc = " Low throttle | Steering:  " + str(self.STEERING_THR4) + " - " + str(self.STEERING_THR5), dynamic_ncols = True)
            self.progress_bar4 = tqdm(total = self.MAX_DATA-self.a, desc = " Low throttle | Steering:  " + str(self.STEERING_THR5) + " - max", dynamic_ncols = True)
            self.progress_bar10 = tqdm(total = self.MAX_DATA-self.kk, desc = "High throttle | Steering: " + str(self.STEERING_THR1) + " - " + str(self.STEERING_THR2), dynamic_ncols = True)
            self.progress_bar11 = tqdm(total = self.MAX_DATA-self.ii, desc = "High throttle | Steering:  " + str(self.STEERING_THR2) + " - " + str(self.STEERING_THR3), dynamic_ncols = True)
            self.progress_bar12 = tqdm(total = self.MAX_DATA-self.jj, desc = "High throttle | Steering:  " + str(self.STEERING_THR3) + " - " + str(self.STEERING_THR4), dynamic_ncols = True)
            self.progress_bar13 = tqdm(total = self.MAX_DATA-self.hh, desc = "High throttle | Steering:  " + str(self.STEERING_THR4) + " - " + str(self.STEERING_THR5), dynamic_ncols = True)
            self.progress_bar14 = tqdm(total = self.MAX_DATA-self.aa, desc = "High throttle | Steering:  " + str(self.STEERING_THR5) + " - max", dynamic_ncols = True)

            
            self.create_subscription(Float32, '/sensing/combination_navigation/chc/pitch', self.pitch_topic_callback, 1)
            self.create_subscription(ActuationStatusStamped, '/vehicle/status/actuation_status', self.actuation_topic_callback, 1)
            self.create_subscription(SteeringReport, '/vehicle/status/steering_status', self.steer_topic_callback, 1)
            self.create_subscription(VelocityReport, '/vehicle/status/velocity_status', self.velocity_topic_callback, 1)
            self.create_subscription(Imu, '/vehicle/status/imu', self.imu_topic_callback, 1)
            self.progress = self.create_publisher(SteeringProcesses, '/scenarios_collection_steering_progress', 10)
            self.timer = self.create_timer(0.02, self.test_callback)

            
        
            
            

            
      # WE WILL FILTER THE DATA DURING THE POST-PROCESSING, SO IN THIS CASE WE DON'T USE DE QUEUE

      def pitch_topic_callback(self, msg):
            
            self.pitch_angle = float(msg.data)
            
                          
                  
      def velocity_topic_callback(self, msg):
            self.velocity = float(msg.longitudinal_velocity)
            


      def actuation_topic_callback(self, msg):
            
            self.throttling = float(msg.status.accel_status)*100.0
                  
                  

      def steer_topic_callback(self, msg):
            
            self.steering = float(msg.steering_tire_angle)
                   
                    

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
            dict1 = {'Velocity': self.vel1, 'Throttling': self.cmd1, 'Steering': self.steer1, 'Acceleration_with_pitch_comp': self.acc1, 'Acceleration_measured': self.accp1, 'Pitch_angle': self.pitch1, 'Index_low_cmd': self.k, "Index_high_cmd": self.kk}
            df1 = pd.DataFrame(dict1)
            df1.to_csv('steering_01.csv') 




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
            dict2 = {'Velocity': self.vel2, 'Throttling': self.cmd2, 'Steering': self.steer2, 'Acceleration_with_pitch_comp': self.acc2, 'Acceleration_measured': self.accp2, 'Pitch_angle': self.pitch2, 'Index_low_cmd': self.i, 'Index_high_cmd': self.ii}
            df2 = pd.DataFrame(dict2)
            df2.to_csv('steering_02.csv') 




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
            dict3 = {'Velocity': self.vel3, 'Throttling': self.cmd3, 'Steering': self.steer3, 'Acceleration_with_pitch_comp': self.acc3, 'Acceleration_measured': self.accp3, 'Pitch_angle': self.pitch3, 'Index_low_cmd': self.j, 'Index_high_cmd': self.jj}
            df3 = pd.DataFrame(dict3)
            df3.to_csv('steering_03.csv') 




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
            dict4 = {'Velocity': self.vel4, 'Throttling': self.cmd4, 'Steering': self.steer4, 'Acceleration_with_pitch_comp': self.acc4, 'Acceleration_measured': self.accp4, 'Pitch_angle': self.pitch4, 'Index_low_cmd': self.h, 'Index_high_cmd': self.hh}
            df4 = pd.DataFrame(dict4)
            df4.to_csv('steering_04.csv') 




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
            dict5 = {'Velocity': self.vel5, 'Throttling': self.cmd5, 'Steering': self.steer5, 'Acceleration_with_pitch_comp': self.acc5, 'Acceleration_measured': self.accp5, 'Pitch_angle': self.pitch5, 'Index_low_cmd': self.a, 'Index_high_cmd': self.aa}
            df5 = pd.DataFrame(dict5)
            df5.to_csv('steering_05.csv') 





      


      def test_callback(self):

            steer_processes_msg = SteeringProcesses()
            steer_progress_msg = SteeringProgress()
            
            
            if(0 < abs(self.velocity) <= self.MAX_VELOCITY):
                  
                  #low throttling scenario

                  if(self.throttling <= self.THROTTLE_THRESHOLD):
                        
                        if(self.STEERING_THR1 <= abs(self.steering) < self.STEERING_THR2 and self.k < self.MAX_DATA):
                              
                              self.collection_steering1()
                              self.progress_bar0.update(1)
                              self.k += 1

                              steer_progress_msg.pedal_value_start = 0
                              steer_progress_msg.pedal_value_end = self.THROTTLE_THRESHOLD
                              steer_progress_msg.steering_value_start = self.STEERING_THR1
                              steer_progress_msg.steering_value_end = self.STEERING_THR2
                              steer_progress_msg.velocity_max = self.MAX_VELOCITY
                              steer_progress_msg.data_count = self.k
                              steer_progress_msg.progress = self.k*100/self.MAX_DATA

                              steer_processes_msg.headers[0] = Header()
                              steer_processes_msg.headers[0].stamp = rospy.Time.from_sec(time.time())
                              steer_processes_msg.headers[0].frame_id = "Steering scenario 1"
                              steer_processes_msg.processes[0] = steer_progress_msg

                              self.progress.publish(steer_processes_msg)

                        
                        elif(self.STEERING_THR2 <= abs(self.steering) < self.STEERING_THR3 and self.i < self.MAX_DATA):
                              
                              self.collection_steering2()
                              self.progress_bar1.update(1)
                              self.i += 1

                              steer_progress_msg.pedal_value_start = 0
                              steer_progress_msg.pedal_value_end = self.THROTTLE_THRESHOLD
                              steer_progress_msg.steering_value_start = self.STEERING_THR2
                              steer_progress_msg.steering_value_end = self.STEERING_THR3
                              steer_progress_msg.velocity_max = self.MAX_VELOCITY
                              steer_progress_msg.data_count = self.i
                              steer_progress_msg.progress = self.i*100/self.MAX_DATA

                              steer_processes_msg.headers[1] = Header()
                              steer_processes_msg.headers[1].stamp = rospy.Time.from_sec(time.time())
                              steer_processes_msg.headers[1].frame_id = "Steering scenario 2"
                              steer_processes_msg.processes[1] = steer_progress_msg

                              self.progress.publish(steer_processes_msg)


                        elif(self.STEERING_THR3 <= abs(self.steering) < self.STEERING_THR4 and self.j < self.MAX_DATA):
                              
                              self.collection_steering3()
                              self.progress_bar2.update(1)
                              self.j += 1

                              steer_progress_msg.pedal_value_start = 0
                              steer_progress_msg.pedal_value_end = self.THROTTLE_THRESHOLD
                              steer_progress_msg.steering_value_start = self.STEERING_THR3
                              steer_progress_msg.steering_value_end = self.STEERING_THR4
                              steer_progress_msg.velocity_max = self.MAX_VELOCITY
                              steer_progress_msg.data_count = self.j
                              steer_progress_msg.progress = self.j*100/self.MAX_DATA

                              steer_processes_msg.headers[2] = Header()
                              steer_processes_msg.headers[2].stamp = rospy.Time.from_sec(time.time())
                              steer_processes_msg.headers[2].frame_id = "Steering scenario 3"
                              steer_processes_msg.processes[2] = steer_progress_msg

                              self.progress.publish(steer_processes_msg)


                        elif(self.STEERING_THR4 <= abs(self.steering) < self.STEERING_THR5 and self.h < self.MAX_DATA):
                              
                              self.collection_steering4()
                              self.progress_bar3.update(1)
                              self.h += 1

                              steer_progress_msg.pedal_value_start = 0
                              steer_progress_msg.pedal_value_end = self.THROTTLE_THRESHOLD
                              steer_progress_msg.steering_value_start = self.STEERING_THR4
                              steer_progress_msg.steering_value_end = self.STEERING_THR5
                              steer_progress_msg.velocity_max = self.MAX_VELOCITY
                              steer_progress_msg.data_count = self.h
                              steer_progress_msg.progress = self.h*100/self.MAX_DATA

                              steer_processes_msg.headers[3] = Header()
                              steer_processes_msg.headers[3].stamp = rospy.Time.from_sec(time.time())
                              steer_processes_msg.headers[3].frame_id = "Steering scenario 4"
                              steer_processes_msg.processes[3] = steer_progress_msg

                              self.progress.publish(steer_processes_msg)


                        elif(abs(self.steering) >= self.STEERING_THR5 and self.a < self.MAX_DATA):

                              self.collection_steering5()
                              self.progress_bar4.update(1)
                              self.a += 1

                              steer_progress_msg.pedal_value_start = 0
                              steer_progress_msg.pedal_value_end = self.THROTTLE_THRESHOLD
                              steer_progress_msg.steering_value_start = self.STEERING_THR5
                              steer_progress_msg.steering_value_end = 0.50
                              steer_progress_msg.velocity_max = self.MAX_VELOCITY
                              steer_progress_msg.data_count = self.a
                              steer_progress_msg.progress = self.a*100/self.MAX_DATA

                              steer_processes_msg.headers[4] = Header()
                              steer_processes_msg.headers[4].stamp = rospy.Time.from_sec(time.time())
                              steer_processes_msg.headers[4].frame_id = "Steering scenario 5"
                              steer_processes_msg.processes[4] = steer_progress_msg

                              self.progress.publish(steer_processes_msg)





                  #high throttling scenario

                  elif(self.throttling > self.THROTTLE_THRESHOLD):
                        
                        if(self.STEERING_THR1 <= abs(self.steering) < self.STEERING_THR2 and self.kk < self.MAX_DATA):
                              
                              self.collection_steering1()
                              self.progress_bar10.update(1)
                              self.kk += 1

                              steer_progress_msg.pedal_value_start = self.THROTTLE_THRESHOLD
                              steer_progress_msg.pedal_value_end = 100
                              steer_progress_msg.steering_value_start = self.STEERING_THR1
                              steer_progress_msg.steering_value_end = self.STEERING_THR2
                              steer_progress_msg.velocity_max = self.MAX_VELOCITY
                              steer_progress_msg.data_count = self.kk
                              steer_progress_msg.progress = self.kk*100/self.MAX_DATA

                              steer_processes_msg.headers[5] = Header()
                              steer_processes_msg.headers[5].stamp = rospy.Time.from_sec(time.time())
                              steer_processes_msg.headers[5].frame_id = "Steering scenario 6"
                              steer_processes_msg.processes[5] = steer_progress_msg

                              self.progress.publish(steer_processes_msg)

                        
                        elif(self.STEERING_THR2 <= abs(self.steering) < self.STEERING_THR3 and self.ii < self.MAX_DATA):
                              
                              self.collection_steering2()
                              self.progress_bar11.update(1)
                              self.ii += 1

                              steer_progress_msg.pedal_value_start = self.THROTTLE_THRESHOLD
                              steer_progress_msg.pedal_value_end = 100
                              steer_progress_msg.steering_value_start = self.STEERING_THR2
                              steer_progress_msg.steering_value_end = self.STEERING_THR3
                              steer_progress_msg.velocity_max = self.MAX_VELOCITY
                              steer_progress_msg.data_count = self.ii
                              steer_progress_msg.progress = self.ii*100/self.MAX_DATA

                              steer_processes_msg.headers[6] = Header()
                              steer_processes_msg.headers[6].stamp = rospy.Time.from_sec(time.time())
                              steer_processes_msg.headers[6].frame_id = "Steering scenario 7"
                              steer_processes_msg.processes[6] = steer_progress_msg

                              self.progress.publish(steer_processes_msg)


                        elif(self.STEERING_THR3 <= abs(self.steering) < self.STEERING_THR4 and self.jj < self.MAX_DATA):
                              
                              self.collection_steering3()
                              self.progress_bar12.update(1)
                              self.jj += 1

                              steer_progress_msg.pedal_value_start = self.THROTTLE_THRESHOLD
                              steer_progress_msg.pedal_value_end = 100
                              steer_progress_msg.steering_value_start = self.STEERING_THR3
                              steer_progress_msg.steering_value_end = self.STEERING_THR4
                              steer_progress_msg.velocity_max = self.MAX_VELOCITY
                              steer_progress_msg.data_count = self.jj
                              steer_progress_msg.progress = self.jj*100/self.MAX_DATA

                              steer_processes_msg.headers[7] = Header()
                              steer_processes_msg.headers[7].stamp = rospy.Time.from_sec(time.time())
                              steer_processes_msg.headers[7].frame_id = "Steering scenario 8"
                              steer_processes_msg.processes[7] = steer_progress_msg

                              self.progress.publish(steer_processes_msg)


                        elif(self.STEERING_THR4 <= abs(self.steering) < self.STEERING_THR5 and self.hh < self.MAX_DATA):
                              
                              self.collection_steering4()
                              self.progress_bar13.update(1)
                              self.hh += 1

                              steer_progress_msg.pedal_value_start = self.THROTTLE_THRESHOLD
                              steer_progress_msg.pedal_value_end = 100
                              steer_progress_msg.steering_value_start = self.STEERING_THR4
                              steer_progress_msg.steering_value_end = self.STEERING_THR5
                              steer_progress_msg.velocity_max = self.MAX_VELOCITY
                              steer_progress_msg.data_count = self.hh
                              steer_progress_msg.progress = self.hh*100/self.MAX_DATA

                              steer_processes_msg.headers[8] = Header()
                              steer_processes_msg.headers[8].stamp = rospy.Time.from_sec(time.time())
                              steer_processes_msg.headers[8].frame_id = "Steering scenario 9"
                              steer_processes_msg.processes[8] = steer_progress_msg

                              self.progress.publish(steer_processes_msg)


                        elif(abs(self.steering) >= self.STEERING_THR5 and self.aa < self.MAX_DATA):

                              self.collection_steering5()
                              self.progress_bar14.update(1)
                              self.aa += 1

                              steer_progress_msg.pedal_value_start = self.THROTTLE_THRESHOLD
                              steer_progress_msg.pedal_value_end = 100
                              steer_progress_msg.steering_value_start = self.STEERING_THR5
                              steer_progress_msg.steering_value_end = 0.50
                              steer_progress_msg.velocity_max = self.MAX_VELOCITY
                              steer_progress_msg.data_count = self.aa
                              steer_progress_msg.progress = self.aa*100/self.MAX_DATA

                              steer_processes_msg.headers[9] = Header()
                              steer_processes_msg.headers[9].stamp = rospy.Time.from_sec(time.time())
                              steer_processes_msg.headers[9].frame_id = "Steering scenario 10"
                              steer_processes_msg.processes[9] = steer_progress_msg

                              self.progress.publish(steer_processes_msg)


                      
                              
                              

                                                    

def main():
      rclpy.init()
      node = primotest()
      rclpy.spin(node)

if __name__ == '__main__':
      main()