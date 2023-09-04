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
- **DATA ACQUISITION: here we collect data from the sensors installed in the vehicle. Specifically, control command cmd is measured from controller area network (CAN bus), speed v is measured from vehicle control unit (VCU), acceleration acc is measured from inertial measurement unit (IMU) and then we should also collect the steering angle, to check if it remains within a threshold we will define later on. It could be useful also to measure the pitch angle of the vehicle for the following reason: due to existence of irregularities of the ground, the pitch angle value will vary during the data collection process. This deviation of the pitch angle adds a component of gravitational acceleration to the longitudinal acceleration, disturbing the real acceleration we want to measure. So, this component should be removed from the acceleration we actually measure according to the following formula: a=a*-gsin(theta). Where a* is the acceleration we measure. Moreover, in this module also data preprocessing and filtering may also be performed to ensure data quality.**










## How to use






