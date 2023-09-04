# Learning Based Vehicle Calibration

## Overview

Here we present the software structure and data collection about the offline calibration, in two different scenarios: normal driving condition and parking condition.

## Input Data Software

It includes:
- **data_type: input file is rosbag(ROS2)**

launch ros node as follows:
```
#!/bin/bash
#source /home/pixkit/pix/pixkit/Autoware/install/setup.bash
#ros2 launch ros2_socketcan socket_can_bridge.launch.xml
#ros2 launch pix_hooke_driver pix_hooke_driver.launch.xml
#ros2 launch pixkit_sensor_kit_launch gnss.launch
gnome-terminal -x bash -c "source /home/pixkit/pix/pixkit/Autoware/install/setup.bash;ros2 launch ros2_socketcan socket_can_bridge.launch.xml; exec bash"
sleep 1
gnome-terminal -x bash -c "source /home/pixkit/pix/pixkit/Autoware/install/setup.bash;ros2 launch pix_hooke_driver pix_hooke_driver.launch.xml; exec bash"
sleep 1
gnome-terminal -x bash -c "source /home/pixkit/pix/pixkit/Autoware/install/setup.bash;ros2 launch pixkit_sensor_kit_launch gnss.launch.xml; exec bash"


```

record data as follows:
```
sleep 1
#/pix_hooke/v2a_brakestafb /pix_hooke/v2a_drivestafb /pix_hooke/v2a_steerstafb /gnss/chc/imu /gnss/chc/pitch
gnome-terminal -x bash -c "source /home/pixkit/pix/pixkit/Autoware/install/setup.bash;ros2 bag record /pix_hooke/v2a_brakestafb /pix_hooke/v2a_drivestafb /pix_hooke/v2a_steerstafb /gnss/chc/imu /gnss/chc/pitch; exec bash"
```


- **data_field: rosbag should contains brake_paddle, throttle_paddle, velocity, steering, imu and pitch**

data contains as follows and how to get value
```
# brake paddle value (frome 0 to 100, float)
/pix_hooke/v2a_brakestafb -> vcu_chassis_brake_padl_fb
# throttel paddle value (frome 0 to 100, float)
/pix_hooke/v2a_drivestafb -> vcu_chassis_throttle_padl_fb
# velocity value (unit is m/s, float)
/pix_hooke/v2a_drivestafb -> vcu_chassis_speed_fb
# steering value (frome -450 to 450, float)
/pix_hooke/v2a_steerstafb -> vcu_chassis_steer_angle_fb
# imu data(unit is rad/s or m/s)
/gnss/chc/imu
# pitch angle (unit is degrees)
/gnss/chc/pitch -> data
```
- **learning-base model in both scenarios: regular scenario and parking scenario**

it contains how scenarios, regular scenario and parking scenario. parking scenario accept steering value at dead slow speed, regular scenario dispose of steering value at normal speed.
## Record Data Software Case

It is instructions for how to manipulate vehicle when collect data, the test site should be as flat as possiable(pitch angle should not over 1 degree). 

1.regular scenario data record - accelerate

vehicle maintain const speed(0-20km/h, 5km/h per step) each time, combain different throttle paddle target(0-50%, 5% per step).

2.regular scenario data record - decelerate

vehicle maintain const speed(20-5km/h, 5km/h per step) each time, combain different brake paddle target(0-50%, 5% per step).

3.regular scenario data record - slide

vehicle maintain const speed(20-5km/h, 5km/h per step) each time, combain 0 throttle paddle value and 0 brake paddle value.

4.parking scenario data record

vehicle maintain const speed(0-20km/h, 5km/h per step) each time, combain different steering angle target(0-100, 5% per step).

## Software Structure
When designing the software, we should ensure modularity, maintainability and scalability.

Here is my proposal of the structure, which could be modified when doing the tests and when implementing the code on ROS2:

- **MAIN FUNCTION: here we initialize components and orchestrate the calibration process.**
- **DATA ACQUISITION: here we collect data from the sensors installed in the vehicle. Specifically, control command cmd is measured from controller area network (CAN bus), speed v is measured from vehicle control unit (VCU), acceleration acc is measured from inertial measurement unit (IMU) and then we should also collect the steering angle, to check if it remains within a threshold we will define later on. It could be useful also to measure the pitch angle of the vehicle for the following reason: due to existence of irregularities of the ground, the pitch angle value will vary during the data collection process. This deviation of the pitch angle adds a component of gravitational acceleration to the longitudinal acceleration, disturbing the real acceleration we want to measure. So, this component should be removed from the acceleration we actually measure. Moreover, in this module also data preprocessing and filtering may also be performed to ensure data quality.**
- **NEURAL NETWORK MODULE: this module is useful for classification tasks. The NN takes preprocessed data as input and outputs classification results. According to the paper we are referring to for developing this project, the best NN model is a standard 3-layer FF NN with a Sigmoid function as activation function and mean square error as the cost function. When testing the algorithm, we could modify this model and try to find a better one for our needs.**
- **PERFORMANCE EVALUATION: : in this module we can evaluate both the offline and online calibration, by testing different models and check whether the online calibration adjusts the offline table, by improving the control performances.**










## How to use
This part will be done after we finish our software and algorithms. Work in progress..






