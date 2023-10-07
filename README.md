# Learning Based Vehicle Calibration

## Overview

Here we present the software structure, data collection, data preprocessing and neural network training and visualization about the offline calibration, in two different scenarios: normal driving condition and parking condition.

## Input Data Software


Launch ros node as follows:
```
./autoware.sh

```

and then record the topics we need to collect in order to train our model. The data we need to record are the pitch angle, the linear acceleration, the velocity, the braking and throttling values and the steering angle:

```
ros2 bag record /sensing/gnss/chc/pitch /pix_robobus/brake_report /pix_robobus/throttle_report /pix_robobus/steering_report /pix_robobus/vcu_report /sensing/gnss/chc/imu

```




- **data_field: rosbag should contains brake_paddle, throttle_paddle, velocity, steering, imu and pitch**

data contains as follows and how to get value
```
# brake paddle value (frome 0 to 100, float)
/pix_robobus/brake_report -> brake_pedal_actual
# throttle paddle value (frome 0 to 100, float)
/pix_robobus/throttle_report -> dirve_throttle_pedal_actual
# velocity value (unit is m/s, float)
/pix_robobus/vcu_report -> vehicle_speed
# steering value (frome -450 to 450, float)
/pix_robobus/steering_report -> steer_angle_actual
# imu data(unit is rad/s or m/s)
/sensing/gnss/chc/imu -> linear_acceleration.x
# pitch angle (unit is degrees)
/sensing/gnss/chc/pitch -> data
```
- **learning-base model in both scenarios: regular scenario and parking scenario**

it contains two scenarios, regular scenario and parking scenario. Parking scenario accept steering value at dead slow speed, regular scenario dispose of steering value at normal speed.

- **run data monitor: data_monotor will check topic node and topic timegap**

```
python3 data_monitor.py
```
## Record Data Software Case

Since our goal is to build a data-driven model, the first step is to collect enough data suitable for training a neural network model. 

Every data is classified according to its speed and throttling/braking information. This way, we can check if we have collected enough data for every case scenario. In order to do so, we have built a software with a simple if-else logic so that it is able to detect and classify the data recorded in its scenario. This stage is very important because this way we can make sure that we have a balanced dataset and that we have enough data to train our neural network model for every conditions. We will start collecting 3000 triplet (velocity, acceleration, throttling/braking) of data per scenario. For each timestamp, if the steering angle is greater than a threshold (2 degrees), that data will be discarded and not classified (since so far we are just interested in the longitudinal dynamic so we should avoid steering):

- **LOW SPEED SCENARIO (0 - 10km/h)**: in this scenario we have 8 different throttling/braking conditions.
0. Brake 0 - deadzone
1. Brake deadzone - 15%
2. Brake 15% - 25%
3. Brake > 25%
4. Throttle 0 - deadzone
5. Throttle deadzone - 30%
6. Throttle 30% - 55%
7. Throttle > 55%

- **HIGH SPEED SCENARIO ( > 10km/h)**: in this scenario we have 8 different throttling/braking conditions.
0. Brake 0 - deadzone
1. Brake deadzone - 15%
2. Brake 15% - 25%
3. Brake > 25%
4. Throttle 0 - deadzone
5. Throttle deadzone - 30%
6. Throttle 30% - 55%
7. Throttle > 55%

As already stated, if the steering angle is greater than 2 degrees, data will not be collected. But there are other conditions under which we discard data:

1. If the velocity is higher than 35km/h, we are not collecting data;
2. If the velocity is equal to 0km/h, we are not collecting data;
3. We need to ensure data consistency. In order to do that, we need to check the values of two consecutive throttle/brake commands: if their difference is greater than a certain threshold, we don't collect those data.

To launch the script:

```
python3 data_collection.py

```
This is how you will visualize the data that are being recorded. This visualization tool updates itself in real time based on the data we are collecting:

![data_collection1](./imgs/data_collection1.png)
![data_collection2](./imgs/data_collection2.png)
![data_collection3](./imgs/data_collection3.png)

This way, while we are manually controlling the vehicle, we can have an idea on how to control it and which maneuver to do in order to fill the entire dataset. Once a case scenario reaches the MAX_DATA threshold, the data in that scenario will not be recorded anymore.


Another important consideration is the pitch compensation:

the longitudinal acceleration we measure from the IMU is not the real longitudinal acceleration. Even if we choose a flat road to collect data, it’s almost impossible to avoid some ground irregularities which are going to create some bumps in the vehicle. These bumps introduce a gravitational acceleration component which disturbs the longitudinal one. Luckily, by measuring the pitch angle, we can remove this disturbance according to the following formula:

real_acc = acc_imu – g * sin(pitch_angle)

where real_acc is the real longitudinal acceleration we want to collect, acc_imu is the longitudinal acceleration measured by the IMU sensor, g is the gravitational acceleration and pitch_angle is the pitch angle measured by the sensor, converted into radians.


These are the rules we adopted for collecting data in an efficient way. But raw data are often affected by noise so before the training stage we should preprocess them. 

## Data Preprocessing

Since raw data collected by human are noisy and non-uniform, preprocessing is an essential step to ensure high quality data.

First, we applied a mean filter to smooth data with a window size of 20 (which can be tuned).

After having collected the data and having saved them in csv format, you can visualize them by launching the following scrips:

```
python3 throttle_data_visualization.py

python3 brake_data_visualization.py

```

This way, we can have an idea about the distribution of our data. 

![throttle_data](./imgs/throttle_data.png)

![brake_data](./imgs/brake_data.png)


As expected, they are noisy and with several outliers, so we need to standardize and normalize them in order to remove outliers and to prepare them for the training stage.

## Neural Network training and visualization

In order to find the relationship between velocity, acceleration and throttle/brake commands, we first need to define our neural network structure and its inputs and output.

Notice that throttling and braking models will be trained separately.

Since we want to generate a calibration table which takes throttle/brake and speed as inputs, and outputs acceleration, we can follow the same structure for the neural network.

So the fully connected neural network will have 2 nodes in the input layer and 1 node in the output layer.

Regarding the hidden layers, we can't know in advance how many layers and parameters we need in order to fit our data in the best way. So, based on the performance we obtain according to some metrics such as MSE, MAE, RMSE and R2, we can modify the structure and the number of parameters to be trained in order to optimize our metrics.

Now we can split the dataset into training set (80%) and testing set (20%) and we are ready for the training stage.

We chose the LogSigmoid function as the activation function, mean square error as the cost function and Adam optimizer, all in Pytorch.

To start the training of the throttling and braking models and visualize their shapes and metrics, we can launch the following scripts:

```
python3 neural_network_throttle.py

python3 neural_network_brake.py

```

![NN_throttle](./imgs/NN_throttle.png)

![NN_brake](./imgs/NN_brake.png)


When you launch these scripts, the training stage begins and when it is done, the software automatically saves the trained model in a zip file.

This way, when you want to use the neural network model, you can avoid to train it everytime from scratch, but you can just train it once and then load it, according to the following scripts:

```
python3 load_throttle_model.py

python3 load_brake_model.py

```

These scripts will create the acceleration and braking offline tables.


Once we have obtained the calibration tables, we can run our vehicle and test their performances by comparing the real velocity of the vehicle with the one tracked by the longitudinal controller (in our case a PID controller). The error between these two velocities should be as close to zero as possible.

We can visualize this error using plotjuggler.







