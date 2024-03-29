import launch
import launch_ros.actions
import os
from launch.actions import OpaqueFunction

def launch_data_monitor_steer(context):
    # Open a new terminal and run data_monitor.py
    os.system("gnome-terminal -- /bin/bash -c 'ros2 run learning_based_vehicle_calibration data_monitor_steer.py; exec bash'")

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'max_data',
            default_value='10000',
            description='Max number of data to collect for each scenario'
        ),
        launch.actions.DeclareLaunchArgument(
            'max_velocity',
            default_value='1.95', 
            description='Max speed in m/s over which we do not collect data'
        ),
        launch.actions.DeclareLaunchArgument(
            'throttle_threshold',
            default_value='12',
            description='Threshold between low and high throttle command'
        ),
        launch.actions.DeclareLaunchArgument(
            'steering_threshold_1',
            default_value='0.04',
            description='Radians value'
        ),
        launch.actions.DeclareLaunchArgument(
            'steering_threshold_2',
            default_value='0.10',
            description='Radians value'
        ),
        launch.actions.DeclareLaunchArgument(
            'steering_threshold_3',
            default_value='0.20', 
            description='Radians value'
        ),
        launch.actions.DeclareLaunchArgument(
            'steering_threshold_4',
            default_value='0.30',
            description='Radians value'
        ),
        launch.actions.DeclareLaunchArgument(
            'steering_threshold_5',
            default_value='0.40',
            description='Radians value'
        ),

        launch.actions.DeclareLaunchArgument(
            'Recovery_Mode',
            default_value='False',
            description='If False, the node will create new csv tables from scratch. If True, it will recover the previous csv tables and will start to collect data from the previous indexes'
        ),


        OpaqueFunction(
            function=launch_data_monitor_steer,
        ),


        launch_ros.actions.Node(
            package='learning_based_vehicle_calibration',
            executable='data_collection_steer.py',
            name='data_collection_steer',
            output='screen',
            parameters=[
                {'max_data': launch.substitutions.LaunchConfiguration('max_data')},
                {'max_velocity': launch.substitutions.LaunchConfiguration('max_velocity')},
                {'throttle_threshold': launch.substitutions.LaunchConfiguration('throttle_threshold')},
                {'steering_threshold_1': launch.substitutions.LaunchConfiguration('steering_threshold_1')},
                {'steering_threshold_2': launch.substitutions.LaunchConfiguration('steering_threshold_2')},
                {'steering_threshold_3': launch.substitutions.LaunchConfiguration('steering_threshold_3')},
                {'steering_threshold_4': launch.substitutions.LaunchConfiguration('steering_threshold_4')},
                {'steering_threshold_5': launch.substitutions.LaunchConfiguration('steering_threshold_5')},

                {'Recovery_Mode': launch.substitutions.LaunchConfiguration('Recovery_Mode')},
            ],

        ),


    ])

    