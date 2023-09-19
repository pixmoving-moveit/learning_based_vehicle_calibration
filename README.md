# Learning Based Vehicle Calibration

## Overview

Here we present the software structure, data collection, data preprocessing and neural network training and visualization about the offline calibration, in two different scenarios: normal driving condition and parking condition.

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

- **run data monitor: data_monotor will check topic node and topic timegap**

```
python3 data_monitor.py
```
## Record Data Software Case

Since our goal is to build a data-driven model, the first step is to collect enough data suitable for training a neural network model. 

Every data is classified according to its speed and throttling/braking information. This way, we can check if we have collected enough data for every case scenario. In order to do so, we have built a software with a simple if-else logic so that it is able to detect and classify the data recorded in its scenario. This stage is very important because this way we can make sure that we have a balanced dataset and that we have enough data to train our neural network model for every conditions. We will start collecting 3000 triplet (velocity, acceleration, throttling/braking) of data per scenario. For each timestamp, if the steering angle is greater than a threshold (2 degrees), that data will be discarded and not classified (since so far we are just interested in the longitudinal dynamic so we should avoid steering):

- **LOW SPEED SCENARIO (0 - 6km/h)**: in this scenario we have 8 different throttling/braking conditions.
0. Brake 0 - deadzone
1. Brake deadzone - 12%
2. Brake 12% - 20%
3. Brake > 20%
4. Throttle 0 - deadzone
5. Throttle deadzone - 20%
6. Throttle 20% - 40%
7. Throttle > 40%

- **HIGH SPEED SCENARIO ( > 6km/h)**: in this scenario we have 8 different throttling/braking conditions.
0. Brake 0 - deadzone
1. Brake deadzone - 12%
2. Brake 12% - 20%
3. Brake > 20%
4. Throttle 0 - deadzone
5. Throttle deadzone - 20%
6. Throttle 20% - 40%
7. Throttle > 40%

As already stated, if the steering angle is greater than 2 degrees, data will not be collected. But there are other conditions under which we discard data:

1. If the velocity is higher than 22km/h, we are not collecting data;
2. If the velocity is equal to 0km/h, we are not collecting data;
3. We need to ensure data consistency. In order to do that, we need to check the values of two consecutive throttle/brake commands: if their difference is greater than a certain threshold, we don't collect those data.

Moreover, we also take into consideration the IMU sensor delay, that is the time between the throttle/brake commands being sent and corresponding acceleration executed. We consider this delay to be about 200ms, but it is a tunable parameter in the software so it is possibile to tune it according to the results you obtain'



These are the rules we adopted for collecting data in an efficient way. But raw data are often affected by noise so before the training stage we should preprocess them. 

## Data Preprocessing

Since raw data collected by human are noisy and non-uniform, preprocessing is an essential step to ensure high quality data.

First, we applied a mean filter to smooth data with a window size of 20 (which can be tuned).

Afterwards, we standardized and normalized data to remove possible outliers and to prepare them for the training stage.

## Neural Network training and visualization

In order to find the relationship between velocity, acceleration and throttle/brake commands, we first need to define our neural network structure and its inputs and output.

Notice that throttling and braking models will be trained separately.

Since we want to generate a calibration table which takes throttle/brake and speed as inputs, and outputs acceleration, we can follow the same structure for the neural network.

So the fully connected neural network will have 2 nodes in the input layer, 64 nodes in the first hidden layer, 16 nodes in the second hidden layer and 1 node in the output layer.

Based on the performance we obtain, we can modify the structure and the number of parameters to be trained.

Now we can split the dataset into training set (80%) and testing set (20%) and we are ready for the training stage.

We chose the Sigmoid function as the activation function, mean square error as the cost function and Adam optimizer, all in Pytorch.





## Software Structure
When designing the software, we should ensure modularity, maintainability and scalability.

Here is my proposal of the structure, which could be modified when doing the tests and when implementing the code on ROS2:

- **MAIN FUNCTION: here we initialize components and orchestrate the calibration process.**
- **DATA ACQUISITION: here we collect data from the sensors installed in the vehicle. Specifically, control command cmd is measured from controller area network (CAN bus), speed v is measured from vehicle control unit (VCU), acceleration acc is measured from inertial measurement unit (IMU) and then we should also collect the steering angle, to check if it remains within a threshold we will define later on. It could be useful also to measure the pitch angle of the vehicle for the following reason: due to existence of irregularities of the ground, the pitch angle value will vary during the data collection process. This deviation of the pitch angle adds a component of gravitational acceleration to the longitudinal acceleration, disturbing the real acceleration we want to measure. So, this component should be removed from the acceleration we actually measure. Moreover, in this module also data preprocessing and filtering may also be performed to ensure data quality.**
- **NEURAL NETWORK MODULE: this module is useful for classification tasks. The NN takes preprocessed data as input and outputs classification results. According to the paper we are referring to for developing this project, the best NN model is a standard 3-layer FF NN with a Sigmoid function as activation function and mean square error as the cost function. When testing the algorithm, we could modify this model and try to find a better one for our needs.**
- **PERFORMANCE EVALUATION: in this module we can evaluate both the offline and online calibration, by testing different models and check whether the online calibration adjusts the offline table, by improving the control performances (later on).**










## How to use
This part will be done after we finish our software and algorithms. Work in progress..






