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

from learning_based_vehicle_calibration.msg import LongitudinalProgress
from learning_based_vehicle_calibration.msg import LongitudinalProcesses

class primotest(rclpy.node.Node):

      def __init__(self):

            super().__init__('primo_test')
            
            
            # data sharing member variables
            self.braking = 0.0
            self.braking_prec = 0.0
            self.throttling = 0.0
            self.throttling_prec = 0.0
            self.steering = 0.0
            self.acceleration = 0.0
            self.velocity = 0.0
            self.pitch_angle = 0.0

            self.g = 9.80665


            # Load params from launch file

            self.MAX_DATA = self.declare_parameter('max_data').get_parameter_value().integer_value
            self.NUM_OF_QUEUE = self.declare_parameter('num_of_queue').get_parameter_value().integer_value
            self.SPEED_THRESHOLD = self.declare_parameter('speed_threshold').get_parameter_value().double_value
            self.STEERING_THRESHOLD = self.declare_parameter('steering_threshold').get_parameter_value().double_value
            self.THROTTLE_DEADZONE = self.declare_parameter('throttle_deadzone').get_parameter_value().integer_value
            self.BRAKE_DEADZONE = self.declare_parameter('brake_deadzone').get_parameter_value().integer_value
            self.MAX_VELOCITY = self.declare_parameter('max_velocity').get_parameter_value().double_value
            self.THROTTLE_THRESHOLD1 = self.declare_parameter('throttle_threshold1').get_parameter_value().integer_value
            self.THROTTLE_THRESHOLD2 = self.declare_parameter('throttle_threshold2').get_parameter_value().integer_value
            self.BRAKE_THRESHOLD1 = self.declare_parameter('brake_threshold1').get_parameter_value().integer_value
            self.BRAKE_THRESHOLD2 = self.declare_parameter('brake_threshold2').get_parameter_value().integer_value
            self.CONSISTENCY_THRESHOLD = self.declare_parameter('consistency_threshold').get_parameter_value().integer_value

            self.RECOVERY_MODE = self.declare_parameter('Recovery_Mode').get_parameter_value().bool_value

            if self.RECOVERY_MODE:
                  df_existing1 = pd.read_csv('throttling.csv')
                  df_existing2 = pd.read_csv('braking.csv')

                  self.k = df_existing1['Low_V_0_deadzone'].iloc[0]
                  self.i = df_existing1['Low_V_deadzone_thr1'].iloc[0]
                  self.j = df_existing1['Low_V_thr1_thr2'].iloc[0]
                  self.h = df_existing1['Low_V_thr2_max'].iloc[0]
                  self.d = df_existing1['High_V_0_deadzone'].iloc[0]
                  self.a = df_existing1['High_V_deadzone_thr1'].iloc[0]
                  self.b = df_existing1['High_V_thr1_thr2'].iloc[0]
                  self.c = df_existing1['High_V_thr2_max'].iloc[0]
                  self.vel = df_existing1['Velocity'].tolist()
                  self.cmd = df_existing1['Throttling'].tolist()
                  self.acc = df_existing1['Acceleration_with_pitch_comp'].tolist()
                  self.acc2 = df_existing1['Acceleration_measured'].tolist()
                  self.pitch = df_existing1['Pitch_angle'].tolist()

                  self.kk = df_existing2['Low_V_0_deadzone'].iloc[0]
                  self.ii = df_existing2['Low_V_deadzone_thr1'].iloc[0]
                  self.jj = df_existing2['Low_V_thr1_thr2'].iloc[0]
                  self.hh = df_existing2['Low_V_thr2_max'].iloc[0]
                  self.dd = df_existing2['High_V_0_deadzone'].iloc[0]
                  self.aa = df_existing2['High_V_deadzone_thr1'].iloc[0]
                  self.bb = df_existing2['High_V_thr1_thr2'].iloc[0]
                  self.cc = df_existing2['High_V_thr2_max'].iloc[0]
                  self.velb = df_existing2['Velocity'].tolist()
                  self.cmdb = df_existing2['Throttling'].tolist()
                  self.accb = df_existing2['Acceleration_with_pitch_comp'].tolist()
                  self.accb2 = df_existing2['Acceleration_measured'].tolist()
                  self.pitch2 = df_existing2['Pitch_angle'].tolist()

            else:
                  self.i = self.j = self.h = self.k = self.a = self.b = self.c = self.d = self.d = self.ii = self.jj = self.hh = self.kk = self.aa = self.bb = self.cc = self.dd = 0
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

                  


            self.progress_bar0 = tqdm(total = self.MAX_DATA-self.k, desc = "Low speed: 0 - Throttle deadzone  ", dynamic_ncols=True)
            self.progress_bar1 = tqdm(total = self.MAX_DATA-self.i, desc = "Low speed: Throttle deadzone - " + str(self.THROTTLE_THRESHOLD1) + " ", dynamic_ncols=True)
            self.progress_bar2 = tqdm(total = self.MAX_DATA-self.j, desc = "Low speed: Throttle " + str(self.THROTTLE_THRESHOLD1) + " - " + str(self.THROTTLE_THRESHOLD2) + "       ", dynamic_ncols=True)
            self.progress_bar3 = tqdm(total = self.MAX_DATA-self.h, desc = "Low speed: Throttle > " + str(self.THROTTLE_THRESHOLD2) + "          ", dynamic_ncols=True)
            self.progress_bar4 = tqdm(total = self.MAX_DATA-self.d, desc = "High speed: 0 - Throttle deadzone ", dynamic_ncols=True)
            self.progress_bar5 = tqdm(total = self.MAX_DATA-self.a, desc = "High speed: Throttle deadzone - " + str(self.THROTTLE_THRESHOLD2), dynamic_ncols=True)
            self.progress_bar6 = tqdm(total = self.MAX_DATA-self.b, desc = "High speed: Throttle " + str(self.THROTTLE_THRESHOLD1) + " - " + str(self.THROTTLE_THRESHOLD2) + "      ", dynamic_ncols=True)
            self.progress_bar7 = tqdm(total = self.MAX_DATA-self.c, desc = "High speed: Throttle > " + str(self.THROTTLE_THRESHOLD2) + "         ", dynamic_ncols=True)
            self.progress_bar8 = tqdm(total = self.MAX_DATA-self.kk, desc = "Low speed: 0 - Brake deadzone     ", dynamic_ncols=True)
            self.progress_bar9 = tqdm(total = self.MAX_DATA-self.ii, desc = "Low speed: Brake deadzone - " + str(self.BRAKE_THRESHOLD1) + "    ", dynamic_ncols=True)
            self.progress_bar10 = tqdm(total = self.MAX_DATA-self.jj, desc = "Low speed: Brake " + str(self.BRAKE_THRESHOLD1) + " - " + str(self.BRAKE_THRESHOLD2) + "          ", dynamic_ncols=True) 
            self.progress_bar11 = tqdm(total = self.MAX_DATA-self.hh, desc = "Low speed: Brake > " + str(self.BRAKE_THRESHOLD2) + "             ", dynamic_ncols=True)
            self.progress_bar12 = tqdm(total = self.MAX_DATA-self.dd, desc = "High speed: 0 - Brake deadzone    ", dynamic_ncols=True)
            self.progress_bar13 = tqdm(total = self.MAX_DATA-self.aa, desc = "High speed: Brake deadzone - " + str(self.BRAKE_THRESHOLD1) + "   ", dynamic_ncols=True)
            self.progress_bar14 = tqdm(total = self.MAX_DATA-self.bb, desc = "High speed: Brake " + str(self.BRAKE_THRESHOLD1) + " - " + str(self.BRAKE_THRESHOLD2) + "         ", dynamic_ncols=True)
            self.progress_bar15 = tqdm(total = self.MAX_DATA-self.cc, desc = "High speed: Brake > " + str(self.BRAKE_THRESHOLD2) + "            ", dynamic_ncols=True)

            
            self.create_subscription(Float32, '/sensing/combination_navigation/chc/pitch', self.pitch_topic_callback, 1)
            self.create_subscription(ActuationStatusStamped, '/vehicle/status/actuation_status', self.actuation_topic_callback, 1)
            self.create_subscription(SteeringReport, '/vehicle/status/steering_status', self.steer_topic_callback, 1)
            self.create_subscription(VelocityReport, '/vehicle/status/velocity_status', self.velocity_topic_callback, 1)
            self.create_subscription(Imu, '/vehicle/status/imu', self.imu_topic_callback, 1)
            self.progress = self.create_publisher(LongitudinalProcesses, '/scenarios_collection_longitudinal_progress', 10)
            self.timer = self.create_timer(0.02, self.test_callback)

            
            
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


      def actuation_topic_callback(self, msg):
            
            self.braking = float(msg.status.brake_status)*100.0
            if(len(self.queue_braking)<self.NUM_OF_QUEUE):
                  self.queue_braking.append(self.braking)
            else:
                  self.queue_braking.popleft()
            self.throttling = float(msg.status.accel_status)*100.0
            if(len(self.queue_throttle)<self.NUM_OF_QUEUE):
                  self.queue_throttle.append(self.throttling)
            else:
                  self.queue_throttle.popleft()
                  

      def steer_topic_callback(self, msg):
            
            self.steering = float(msg.steering_tire_angle)
            
            

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
            dict1 = {'Velocity': self.vel, 'Throttling': self.cmd, 'Acceleration_with_pitch_comp': self.acc, 'Acceleration_measured': self.acc2, 'Pitch_angle': self.pitch, 'Low_V_0_deadzone': self.k, 'Low_V_deadzone_thr1': self.i, 'Low_V_thr1_thr2': self.j, 'Low_V_thr2_max': self.h, 'High_V_0_deadzone': self.d, 'High_V_deadzone_thr1': self.a, 'High_V_thr1_thr2': self.b, 'High_V_thr2_max': self.c}
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
                              
                              
                              
            dict2 = {'Velocity': self.velb, 'Braking': self.cmdb, 'Acceleration_with_pitch_comp': self.accb, 'Acceleration_measured': self.accb2, 'Pitch_angle': self.pitch2, 'Low_V_0_deadzone': self.kk, 'Low_V_deadzone_thr1': self.ii, 'Low_V_thr1_thr2': self.jj, 'Low_V_thr2_max': self.hh, 'High_V_0_deadzone': self.dd, 'High_V_deadzone_thr1': self.aa, 'High_V_thr1_thr2': self.bb, 'High_V_thr2_max': self.cc}
            df2 = pd.DataFrame(dict2)
            df2.to_csv('braking.csv') 

            self.braking_prec = mean(self.queue_braking)
            
            
            
            
            
                  
                        
                  
      


      def test_callback(self):

            long_processes_msg = LongitudinalProcesses()
            long_progress_msg = LongitudinalProgress()

            if(len(self.queue_throttle)>=self.NUM_OF_QUEUE and(len(self.queue_braking)>=self.NUM_OF_QUEUE)):

                  # THROTTLING SCENARIO to train throttling model
                  if(self.braking == 0 and abs(self.steering) < self.STEERING_THRESHOLD and abs(self.throttling_prec-mean(self.queue_throttle)) <= self.CONSISTENCY_THRESHOLD):
                        
                        #low velocity scenario

                        if(0 < abs(self.velocity) <= self.SPEED_THRESHOLD):
                              
                              if(0 <= self.throttling <= self.THROTTLE_DEADZONE and self.k < self.MAX_DATA and self.flag == 0):
                                    
                                    self.collection_throttling()
                                    self.progress_bar0.update(1)
                                    self.k += 1

                                    long_progress_msg.pedal_value_start = 0
                                    long_progress_msg.pedal_value_end = self.THROTTLE_DEADZONE
                                    long_progress_msg.velocity_start = 0
                                    long_progress_msg.velocity_end = self.SPEED_THRESHOLD
                                    long_progress_msg.data_count = self.k
                                    long_progress_msg.progress = self.k*100/self.MAX_DATA

                                    long_processes_msg.headers[0] = Header()
                                    long_processes_msg.headers[0].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[0].frame_id = "Throttle scenario 1"
                                    long_processes_msg.processes[0] = long_progress_msg

                                    self.progress.publish(long_processes_msg)



                              
                              elif(self.THROTTLE_DEADZONE < self.throttling <= self.THROTTLE_THRESHOLD1 and self.i < self.MAX_DATA):
                                    
                                    self.collection_throttling()
                                    self.progress_bar1.update(1)
                                    self.flag = 0
                                    self.i += 1

                                    long_progress_msg.pedal_value_start = self.THROTTLE_DEADZONE
                                    long_progress_msg.pedal_value_end = self.THROTTLE_THRESHOLD1
                                    long_progress_msg.velocity_start = 0
                                    long_progress_msg.velocity_end = self.SPEED_THRESHOLD
                                    long_progress_msg.data_count = self.i
                                    long_progress_msg.progress = self.i*100/self.MAX_DATA

                                    long_processes_msg.headers[1] = Header()
                                    long_processes_msg.headers[1].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[1].frame_id = "Throttle scenario 2"
                                    long_processes_msg.processes[1] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


                              elif(self.THROTTLE_THRESHOLD1 < self.throttling <= self.THROTTLE_THRESHOLD2 and self.j < self.MAX_DATA):
                                    
                                    self.collection_throttling()
                                    self.progress_bar2.update(1)
                                    self.flag = 0
                                    self.j += 1

                                    long_progress_msg.pedal_value_start = self.THROTTLE_THRESHOLD1
                                    long_progress_msg.pedal_value_end = self.THROTTLE_THRESHOLD2
                                    long_progress_msg.velocity_start = 0
                                    long_progress_msg.velocity_end = self.SPEED_THRESHOLD
                                    long_progress_msg.data_count = self.j
                                    long_progress_msg.progress = self.j*100/self.MAX_DATA

                                    long_processes_msg.headers[2] = Header()
                                    long_processes_msg.headers[2].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[2].frame_id = "Throttle scenario 3"
                                    long_processes_msg.processes[2] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


                              elif(self.throttling > self.THROTTLE_THRESHOLD2 and self.h < self.MAX_DATA):
                                    
                                    self.collection_throttling()
                                    self.progress_bar3.update(1)
                                    self.flag = 0
                                    self.h += 1

                                    long_progress_msg.pedal_value_start = self.THROTTLE_THRESHOLD2
                                    long_progress_msg.pedal_value_end = 100
                                    long_progress_msg.velocity_start = 0
                                    long_progress_msg.velocity_end = self.SPEED_THRESHOLD
                                    long_progress_msg.data_count = self.h
                                    long_progress_msg.progress = self.hh*100/self.MAX_DATA

                                    long_processes_msg.headers[3] = Header()
                                    long_processes_msg.headers[3].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[3].frame_id = "Throttle scenario 4"
                                    long_processes_msg.processes[3] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


                        #high velocity scenario

                        elif(self.SPEED_THRESHOLD < abs(self.velocity) <= self.MAX_VELOCITY):
                              
                              if(0 <= self.throttling <= self.THROTTLE_DEADZONE and self.d < self.MAX_DATA and self.flag == 0):
                                    
                                    self.collection_throttling()
                                    self.progress_bar4.update(1)
                                    self.d += 1

                                    long_progress_msg.pedal_value_start = 0
                                    long_progress_msg.pedal_value_end = self.THROTTLE_DEADZONE
                                    long_progress_msg.velocity_start = self.SPEED_THRESHOLD
                                    long_progress_msg.velocity_end = self.MAX_VELOCITY
                                    long_progress_msg.data_count = self.d
                                    long_progress_msg.progress = self.d*100/self.MAX_DATA

                                    long_processes_msg.headers[4] = Header()
                                    long_processes_msg.headers[4].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[4].frame_id = "Throttle scenario 5"
                                    long_processes_msg.processes[4] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


                              elif(self.THROTTLE_DEADZONE < self.throttling <= self.THROTTLE_THRESHOLD1 and self.a < self.MAX_DATA):
                                    
                                    self.collection_throttling()
                                    self.progress_bar5.update(1)
                                    self.flag = 0
                                    self.a += 1

                                    long_progress_msg.pedal_value_start = self.THROTTLE_DEADZONE
                                    long_progress_msg.pedal_value_end = self.THROTTLE_THRESHOLD1
                                    long_progress_msg.velocity_start = self.SPEED_THRESHOLD
                                    long_progress_msg.velocity_end = self.MAX_VELOCITY
                                    long_progress_msg.data_count = self.a
                                    long_progress_msg.progress = self.a*100/self.MAX_DATA

                                    long_processes_msg.headers[5] = Header()
                                    long_processes_msg.headers[5].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[5].frame_id = "Throttle scenario 6"
                                    long_processes_msg.processes[5] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


                              elif(self.THROTTLE_THRESHOLD1 < self.throttling <= self.THROTTLE_THRESHOLD2 and self.b < self.MAX_DATA):
                                    
                                    self.collection_throttling()
                                    self.progress_bar6.update(1)
                                    self.flag = 0
                                    self.b += 1

                                    long_progress_msg.pedal_value_start = self.THROTTLE_THRESHOLD1
                                    long_progress_msg.pedal_value_end = self.THROTTLE_THRESHOLD2
                                    long_progress_msg.velocity_start = self.SPEED_THRESHOLD
                                    long_progress_msg.velocity_end = self.MAX_VELOCITY
                                    long_progress_msg.data_count = self.b
                                    long_progress_msg.progress = self.b*100/self.MAX_DATA

                                    long_processes_msg.headers[6] = Header()
                                    long_processes_msg.headers[6].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[6].frame_id = "Throttle scenario 7"
                                    long_processes_msg.processes[6] = long_progress_msg

                                    self.progress.publish(long_processes_msg)

                              elif(self.throttling > self.THROTTLE_THRESHOLD2 and self.c < self.MAX_DATA):
                                    
                                    self.collection_throttling()
                                    self.progress_bar7.update(1)
                                    self.flag = 0
                                    self.c += 1

                                    long_progress_msg.pedal_value_start = self.THROTTLE_THRESHOLD2
                                    long_progress_msg.pedal_value_end = 100
                                    long_progress_msg.velocity_start = self.SPEED_THRESHOLD
                                    long_progress_msg.velocity_end = self.MAX_VELOCITY
                                    long_progress_msg.data_count = self.c
                                    long_progress_msg.progress = self.c*100/self.MAX_DATA

                                    long_processes_msg.headers[7] = Header()
                                    long_processes_msg.headers[7].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[7].frame_id = "Throttle scenario 8"
                                    long_processes_msg.processes[7] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


                  

                  # BRAKING SCENARIO to train braking model

                  if(self.throttling == 0 and abs(self.steering) < self.STEERING_THRESHOLD and abs(self.braking_prec-mean(self.queue_braking)) <= self.CONSISTENCY_THRESHOLD):
                        
                        #low velocity scenario

                        if(0 < abs(self.velocity) <= self.SPEED_THRESHOLD):
                              
                              if(0 <= self.braking <= self.BRAKE_DEADZONE and self.kk < self.MAX_DATA and self.flag == 1):
                                    
                                    self.collection_braking()
                                    self.progress_bar8.update(1)
                                    self.kk += 1

                                    long_progress_msg.pedal_value_start = 0
                                    long_progress_msg.pedal_value_end = self.BRAKE_DEADZONE
                                    long_progress_msg.velocity_start = 0
                                    long_progress_msg.velocity_end = self.SPEED_THRESHOLD
                                    long_progress_msg.data_count = self.kk
                                    long_progress_msg.progress = self.kk*100/self.MAX_DATA

                                    long_processes_msg.headers[8] = Header()
                                    long_processes_msg.headers[8].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[8].frame_id = "Brake scenario 1"
                                    long_processes_msg.processes[8] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


                              elif(self.BRAKE_DEADZONE < self.braking <= self.BRAKE_THRESHOLD1 and self.ii < self.MAX_DATA):
                                    
                                    self.collection_braking()
                                    self.progress_bar9.update(1)
                                    self.flag = 1
                                    self.ii += 1

                                    long_progress_msg.pedal_value_start = self.BRAKE_DEADZONE
                                    long_progress_msg.pedal_value_end = self.BRAKE_THRESHOLD1
                                    long_progress_msg.velocity_start = 0
                                    long_progress_msg.velocity_end = self.SPEED_THRESHOLD
                                    long_progress_msg.data_count = self.ii
                                    long_progress_msg.progress = self.ii*100/self.MAX_DATA

                                    long_processes_msg.headers[9] = Header()
                                    long_processes_msg.headers[9].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[9].frame_id = "Brake scenario 2"
                                    long_processes_msg.processes[9] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


                              elif(self.BRAKE_THRESHOLD1 < self.braking <= self.BRAKE_THRESHOLD2 and self.jj < self.MAX_DATA):
                                    
                                    self.collection_braking()
                                    self.progress_bar10.update(1)
                                    self.flag = 1
                                    self.jj += 1

                                    long_progress_msg.pedal_value_start = self.BRAKE_THRESHOLD1
                                    long_progress_msg.pedal_value_end = self.BRAKE_THRESHOLD2
                                    long_progress_msg.velocity_start = 0
                                    long_progress_msg.velocity_end = self.SPEED_THRESHOLD
                                    long_progress_msg.data_count = self.jj
                                    long_progress_msg.progress = self.jj*100/self.MAX_DATA

                                    long_processes_msg.headers[10] = Header()
                                    long_processes_msg.headers[10].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[10].frame_id = "Brake scenario 3"
                                    long_processes_msg.processes[10] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


                              elif(self.braking > self.BRAKE_THRESHOLD2 and self.hh < self.MAX_DATA):
                                    
                                    self.collection_braking()
                                    self.progress_bar11.update(1)
                                    self.flag = 1
                                    self.hh += 1

                                    long_progress_msg.pedal_value_start = self.BRAKE_THRESHOLD2
                                    long_progress_msg.pedal_value_end = 100
                                    long_progress_msg.velocity_start = 0
                                    long_progress_msg.velocity_end = self.SPEED_THRESHOLD
                                    long_progress_msg.data_count = self.hh
                                    long_progress_msg.progress = self.hh*100/self.MAX_DATA

                                    long_processes_msg.headers[11] = Header()
                                    long_processes_msg.headers[11].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[11].frame_id = "Brake scenario 4"
                                    long_processes_msg.processes[11] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


                        #high velocity scenario

                        elif(self.SPEED_THRESHOLD < abs(self.velocity) <= self.MAX_VELOCITY):
                              
                              if(0 <= self.braking <= self.BRAKE_DEADZONE and self.dd < self.MAX_DATA and self.flag == 1):
                                    
                                    self.collection_braking()
                                    self.progress_bar12.update(1)
                                    self.dd += 1

                                    long_progress_msg.pedal_value_start = 0
                                    long_progress_msg.pedal_value_end = self.BRAKE_DEADZONE
                                    long_progress_msg.velocity_start = self.SPEED_THRESHOLD
                                    long_progress_msg.velocity_end = self.MAX_VELOCITY
                                    long_progress_msg.data_count = self.dd
                                    long_progress_msg.progress = self.dd*100/self.MAX_DATA

                                    long_processes_msg.headers[12] = Header()
                                    long_processes_msg.headers[12].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[12].frame_id = "Brake scenario 5"
                                    long_processes_msg.processes[12] = long_progress_msg

                                    self.progress.publish(long_processes_msg)

                              elif(self.BRAKE_DEADZONE < self.braking <= self.BRAKE_THRESHOLD1 and self.aa < self.MAX_DATA):
                                    
                                    self.collection_braking()
                                    self.progress_bar13.update(1)
                                    self.flag = 1
                                    self.aa += 1

                                    long_progress_msg.pedal_value_start = self.BRAKE_DEADZONE
                                    long_progress_msg.pedal_value_end = self.BRAKE_THRESHOLD1
                                    long_progress_msg.velocity_start = self.SPEED_THRESHOLD
                                    long_progress_msg.velocity_end = self.MAX_VELOCITY
                                    long_progress_msg.data_count = self.aa
                                    long_progress_msg.progress = self.aa*100/self.MAX_DATA

                                    long_processes_msg.headers[13] = Header()
                                    long_processes_msg.headers[13].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[13].frame_id = "Brake scenario 6"
                                    long_processes_msg.processes[13] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


                              elif(self.BRAKE_THRESHOLD1 < self.braking <= self.BRAKE_THRESHOLD2 and self.bb < self.MAX_DATA):
                                    
                                    self.collection_braking()
                                    self.progress_bar14.update(1)
                                    self.flag = 1
                                    self.bb += 1

                                    long_progress_msg.pedal_value_start = self.BRAKE_THRESHOLD1
                                    long_progress_msg.pedal_value_end = self.BRAKE_THRESHOLD2
                                    long_progress_msg.velocity_start = self.SPEED_THRESHOLD
                                    long_progress_msg.velocity_end = self.MAX_VELOCITY
                                    long_progress_msg.data_count = self.bb
                                    long_progress_msg.progress = self.bb*100/self.MAX_DATA

                                    long_processes_msg.headers[14] = Header()
                                    long_processes_msg.headers[14].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[14].frame_id = "Brake scenario 7"
                                    long_processes_msg.processes[14] = long_progress_msg

                                    self.progress.publish(long_processes_msg)
                                    

                              elif(self.braking > self.BRAKE_THRESHOLD2 and self.cc < self.MAX_DATA):
                                    
                                    self.collection_braking()
                                    self.progress_bar15.update(1)
                                    self.flag = 1  
                                    self.cc += 1  

                                    long_progress_msg.pedal_value_start = self.BRAKE_THRESHOLD2
                                    long_progress_msg.pedal_value_end = 100
                                    long_progress_msg.velocity_start = self.SPEED_THRESHOLD
                                    long_progress_msg.velocity_end = self.MAX_VELOCITY
                                    long_progress_msg.data_count = self.cc
                                    long_progress_msg.progress = self.cc*100/self.MAX_DATA

                                    long_processes_msg.headers[15] = Header()
                                    long_processes_msg.headers[15].stamp = rospy.Time.from_sec(time.time())
                                    long_processes_msg.headers[15].frame_id = "Brake scenario 8"
                                    long_processes_msg.processes[15] = long_progress_msg

                                    self.progress.publish(long_processes_msg)


            
            

                                                                          

def main():
      rclpy.init()
      node = primotest()
      rclpy.spin(node)

if __name__ == '__main__':
      main()