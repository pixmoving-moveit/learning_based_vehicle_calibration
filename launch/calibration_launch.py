import launch
import launch_ros.actions
import os
from launch.actions import OpaqueFunction


def launch_data_monitor():
    # Open a new terminal and run data_monitor.py
    os.system("gnome-terminal -- /bin/bash -c 'ros2 run learning_based_vehicle_calibration data_monitor.py; exec bash'")

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'max_data',
            default_value='1500',
            description='Max number of data to collect for each scenario'
        ),
        launch.actions.DeclareLaunchArgument(
            'num_of_queue',
            default_value='20', 
            description='Window size of mean filter to smooth data'
        ),
        launch.actions.DeclareLaunchArgument(
            'speed_threshold',
            default_value='10.0/3.6', 
            description='Threshold between low and high speeds in m/s'
        ),
        launch.actions.DeclareLaunchArgument(
            'steering_threshold',
            default_value='0.03490658503988659',
            description='Radians value which corresponds to 2 degrees, over which we do not collect data'
        ),
        launch.actions.DeclareLaunchArgument(
            'throttle_deadzone',
            default_value='5',
            description='Percentage of throttle deadzone'
        ),
        launch.actions.DeclareLaunchArgument(
            'brake_deadzone',
            default_value='5',
            description='Percentage of break deadzone'
        ),
        launch.actions.DeclareLaunchArgument(
            'max_velocity',
            default_value='40.0/3.6', 
            description='Max speed in m/s over which we do not collect data'
        ),
        launch.actions.DeclareLaunchArgument(
            'throttle_threshold1',
            default_value='30',
            description='Threshold throttle percentage 1'
        ),
        launch.actions.DeclareLaunchArgument(
            'throttle_threshold2',
            default_value='55',
            description='Threshold throttle percentage 2'
        ),
        launch.actions.DeclareLaunchArgument(
            'brake_threshold1',
            default_value='15',
            description='Threshold brake percentage 1'
        ),
        launch.actions.DeclareLaunchArgument(
            'brake_threshold2',
            default_value='25',
            description='Threshold brake percentage 2'
        ),
        launch.actions.DeclareLaunchArgument(
            'consistency_threshold',
            default_value='20',
            description='If 2 consecutive throttle or brake commands differ for more than 20, they are not consistent so we do not collect them'
        ),

        launch.actions.DeclareLaunchArgument(
            'Recovery_Mode',
            default_value='False',
            description='If False, the node will create new csv tables from scratch. If True, it will recover the previous csv tables and will start to collect data from the previous indexes'
        ),


        OpaqueFunction(
            function=launch_data_monitor,
        ),


        launch_ros.actions.Node(
            package='learning_based_vehicle_calibration',
            executable='data_collection.py',
            name='data_collection',
            output='screen',
            parameters=[
                {'max_data': launch.substitutions.LaunchConfiguration('max_data')},
                {'num_of_queue': launch.substitutions.LaunchConfiguration('num_of_queue')},
                {'speed_threshold': launch.substitutions.LaunchConfiguration('speed_threshold')},
                {'steering_threshold': launch.substitutions.LaunchConfiguration('steering_threshold')},
                {'throttle_deadzone': launch.substitutions.LaunchConfiguration('throttle_deadzone')},
                {'brake_deadzone': launch.substitutions.LaunchConfiguration('brake_deadzone')},
                {'max_velocity': launch.substitutions.LaunchConfiguration('max_velocity')},
                {'throttle_threshold1': launch.substitutions.LaunchConfiguration('throttle_threshold1')},
                {'throttle_threshold2': launch.substitutions.LaunchConfiguration('throttle_threshold2')},
                {'brake_threshold1': launch.substitutions.LaunchConfiguration('brake_threshold2')},
                {'brake_threshold2': launch.substitutions.LaunchConfiguration('brake_threshold2')},
                {'consistency_threshold': launch.substitutions.LaunchConfiguration('consistency_threshold')},

                {'Recovery_Mode': launch.substitutions.LaunchConfiguration('Recovery_Mode')},
            ],

        ),


    ])

        
            