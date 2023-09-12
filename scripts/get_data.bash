#!/bin/bash
#source /home/pixkit/pix/pixkit/Autoware/install/setup.bash
#ros2 launch ros2_socketcan socket_can_bridge.launch.xml
#ros2 launch pix_hooke_driver pix_hooke_driver.launch.xml
#ros2 launch pixkit_sensor_kit_launch gnss.launch.xml
gnome-terminal -x bash -c "source /home/pixkit/pix/pixkit/Autoware/install/setup.bash;ros2 launch ros2_socketcan socket_can_bridge.launch.xml; exec bash"
sleep 1
gnome-terminal -x bash -c "source /home/pixkit/pix/pixkit/Autoware/install/setup.bash;ros2 launch pix_hooke_driver pix_hooke_driver.launch.xml; exec bash"
sleep 1
gnome-terminal -x bash -c "source /home/pixkit/pix/pixkit/Autoware/install/setup.bash;ros2 launch pixkit_sensor_kit_launch gnss.launch.xml; exec bash"
sleep 1
#/pix_hooke/v2a_brakestafb /pix_hooke/v2a_drivestafb /pix_hooke/v2a_steerstafb /gnss/chc/imu /gnss/chc/pitch
gnome-terminal -x bash -c "source /home/pixkit/pix/pixkit/Autoware/install/setup.bash;ros2 bag record /pix_hooke/v2a_brakestafb /pix_hooke/v2a_drivestafb /pix_hooke/v2a_steerstafb /gnss/chc/imu /gnss/chc/pitch; exec bash"
