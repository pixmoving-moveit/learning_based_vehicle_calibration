# Learning Based Vehicle Calibration

## Overview

Here we present the software structure and data collection about the offline calibration, in two different scenarios: normal driving condition and parking condition.

## Input Data Software

It includes:
- **data_type**
- **data_field**
- **learning-base model in both scenarios**


## Record Data Software Case

It includes instructions on how to collect data.

## Software Structure
When designing the software, we should ensure modularity, maintainability and scalability.

Here is my proposal of the structure, which could be modified when doing the tests and when implementing the code on ROS2:

- **MAIN FUNCTION: here we initialize components and orchestrate the calibration process.**
- **DATA ACQUISITION: here we collect data from the sensors installed in the vehicle. Specifically, control command cmd is measured from controller area network (CAN bus), speed v is measured from vehicle control unit (VCU), acceleration acc is measured from inertial measurement unit (IMU) and then we should also collect the steering angle, to check if it remains within a threshold we will define later on. It could be useful also to measure the pitch angle of the vehicle for the following reason: due to existence of irregularities of the ground, the pitch angle value will vary during the data collection process. This deviation of the pitch angle adds a component of gravitational acceleration to the longitudinal acceleration, disturbing the real acceleration we want to measure. So, this component should be removed from the acceleration we actually measure. Moreover, in this module also data preprocessing and filtering may also be performed to ensure data quality.**
- **NEURAL NETWORK MODULE: this module is useful for classification tasks. The NN takes preprocessed data as input and outputs classification results. According to the paper we are referring to for developing this project, the best NN model is a standard 3-layer FF NN with a Sigmoid function as activation function and mean square error as the cost function. When testing the algorithm, we could modify this model and try to find a better one for our needs.**
- **PERFORMANCE EVALUATION: : in this module we can evaluate both the offline and online calibration, by testing different models and check whether the online calibration adjusts the offline table, by improving the control performances.**










## How to use






