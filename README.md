# Learning Based Vehicle Calibration

## Overview

Here we present the software structure, data collection, data preprocessing and neural network training and visualization about the end-to-end calibration of a vehicle, in two different scenarios: 
- **Normal driving conditions (longitudinal dynamics);**
- **Parking conditions (lateral dynamics).**

## Input Data Software


Launch Autoware as follows:
```
./autoware.sh
```

and then record the topics we need to collect in order to train our model. The data we need to record are the pitch angle, the linear acceleration, the velocity, the braking and throttling values and the steering angle:

```
ros2 bag record /sensing/gnss/chc/pitch /actuation_input /velocity_input /sensing/gnss/chc/imu
```




- **data_field: rosbag should contains brake_paddle, throttle_paddle, velocity, steering, imu and pitch**

data contains as follows and how to get value
```
# brake paddle value (frome 0 to 100, float)
/actuation_input -> brake_cmd
# throttle paddle value (frome 0 to 100, float)
/actuation_input -> accel_cmd
# velocity value (unit is m/s, float)
/velocity_input -> longitudinal_velocity
# steering value (frome -1 to 1, float)
/actuation_input -> steer_cmd
# imu data(unit is rad/s or m/s)
/sensing/gnss/chc/imu -> linear_acceleration.x
# pitch angle (unit is degrees)
/sensing/gnss/chc/pitch -> data
```



- **run data monitor: data_monotor will check topic node and topic timegap**

```
python3 data_monitor.py
```

By running data_monitor, we can make sure that we are receiving all the topics correctly, without any delay and any problem.

You can have a look at the following examples:

- if all the topics are published correctly, the scripts is going to print to terminal the following text, with the frequency of one second

![data_monitor_normal](./imgs/data_monitor_normal.png)

- if instead one or more topics are not published or received correctly, or they are delayed, the script is going to warn you by printing to terminal for example the following text

![data_monitor_error](./imgs/data_monitor_error.png)



# 1. Longitudinal Dynamics

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

## Neural Network Training and Visualization

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


## Evaluation and Results

In order to evaluate our calibration table for the longitudinal dynamic, we are going to check the speed error of the vehicle while driving in autonomous driving mode.

We have tested the vehicle in different road conditions with different slopes, and these are the results we have obtained:

![evaluation](./imgs/evaluation.png)

The blue line is the speed error [m/s] and the red line is the pitch angle [degrees]. 

As you can see, the speed error is bounded between ± 0.3m/s, which is pretty good considering that the pitch angle is not constant during the test.

We are now ready for the steering calibration tables.

# 2. Lateral Dynamics

Similarly to the Longitudinal Dynamics case, in this module we are going to collect data and train a neural network model to map velocity, throttling command and steering angle into acceleration. Thus, we will have another input, which is the steering angle.

The goal indeed is to calibrate the vehicle's dynamics for low speeds and large steering angles (typically in parking scenarios), and try to obtain a model which can compensate the friction force between the tyres and the road.
Precisely, when the vehicle has a low speed (we assume less than 5km/h), the larger the steering angle, the higher the friction force so we need a higher throttling command to move the vehicle.

For this reason, in these conditions, we cannot rely on the calibration map obtained from the Longitudinal Dynamics case, but we need to build new maps that can take into account this friction compensation strategy.

Here we propose our methods and results.

The idea is to obtain different calibration tables, according to some steering angle ranges. In the Longitudinal Dynamics case, we just obtained one calibration table for the throttling scenario and one calibration table for the braking scenario. In this case, instead, we will obtain 5 calibration tables for the throttling scenario. Each calibration table differs from each other according to the steering range they belong to.

For example, the first calibration table is valid for steering angles between 5% and 20%, the second one is valid for steering angles between 20% and 40% etc.


## Record Data Software Case

Here we follow the same logic of the Longitudinal Dynamics case. The difference is that we have different scenarios:

- **LOW THROTTLE SCENARIO ( <= 12 )**: in this scenario we have 5 different steering conditions.

0. Steering: 5% - 20%
1. Steering: 20% - 40%
2. Steering: 40% - 60%
3. Steering: 60% - 80%
4. Steering: 80% - 100%

- **HIGH THROTTLE SCENARIO ( > 12 )**: in this scenario we have 5 different steering conditions.

0. Steering: 5% - 20%
1. Steering: 20% - 40%
2. Steering: 40% - 60%
3. Steering: 60% - 80%
4. Steering: 80% - 100%


We don't collect data in the following cases:

1. If the velocity is higher than 5km/h;
2. If the velocity is equal to 0km/h;
3. If the steering angle is less than 5%.

To launch the script:

```
python3 data_collection_steer.py
```

This is how you will visualize the data that are being recorded. This visualization tool updates itself in real time based on the data we are collecting:

![data_collection_steer](./imgs/data_collection_steer.png)


The considerations we have done for the longitudinal dynamics case are still valid here.

At the end of the recording process, the script will save our data in 5 different csv files, according to the 5 different steering ranges.

## Data Preprocessing

First, let's visualize our raw data by launching the script (make sure to read the right csv file):

```
python3 steering_data_visualization.py
```

![rawdatasteer](./imgs/raw_data_steer.png)

These are the data of the steering condition: 60% - 80%.

The preprocessing step consist of a mean filter with size 11 (tunable) and the removal of the outliers. Then we standardize and normalize them to be ready for the training stage.


## Neural Network Training and Visualization

Like the Longitudinal Dynamics case, here we find the relationship between velocity, acceleration and throttling command.

We will train 5 different models according to the steering range. The inputs of the neural network model are velocity and throttling command, and the output is the acceleration.

Like before, we chose the LogSigmoid activation function, mean square error as the cost function and Adam optimizer, all in Pytorch.

To start the training stage, visualize the model and save the calibration table in csv format, launch the following script (make sure to read the right csv file):

```
python3 neural_network_steer.py
```

As stated above, you need to train 5 different models, so you will obtain 5 different calibration tables, according to the steering range.

Here is an example of the visualization for the steering range 5% - 20%:

![NNsteer0](./imgs/NN_steer0.png)

Instead, the model for the steering range 60% - 80% looks like this:

![NNsteer](./imgs/NN_steer.png)

As you can see, for the same throttle value, the acceleration is higher is the calibration table for the steering range 5% - 20%. This happens because as already stated, the higher the steering angle, the higher the friction force, so we need a higher throttle command to output the same acceleration value.


## Evaluation and Results



In order to evaluate the performances of our tables in steering conditions at low speeds, we are going to plot the speed error while driving in autonomous driving mode.

First, we will show the speed error of the calibration table without friction compensation:

![err1](./imgs/err1.png)

As you can notice, the error is pretty large, and most important thing, the velocity of the vehicle cannot reach the setpoint. Indeed, the speed error has a bias of around 0.2m/s and cannot reach 0. This happens because this table is not able to compensate the force generated by the friction between the tyres and the ground with large steering angles, so the throttle output is not enough to reach the desired speed.

Let's now visualize the speed error of the calibration tables with friction compensation:

![err3](./imgs/err3.png)

The results are outstanding! As you can see, the speed error is bounded between ± 0.05m/s in large steering conditions at low speeds (such as in parking scenarios).

So, we can conclude that these tables can compensate the friction force and can track the desired speed without problems.