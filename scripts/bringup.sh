#!/bin/bash

#################

# usage:  ./bringup.sh [password] [user@host]

#################

echo "Bringing up robot base"
terminator --new-tab -x "sshpass -p '$1' ssh -t '$2' \
			'source ~/catkin_ws/devel/setup.bash; \
			 echo '$1' | sudo -S chmod 777 /dev/ttyACM0; \
			 roslaunch remote_robot_android car.launch; \
			 exec bash'";

echo "Initialize SOSS: ROS2 -> ROS1 for /cmd_vel"
terminator --new-tab -x "sshpass -p '$1' ssh -t '$2' \
			'source ~/rmf/build/soss/setup.bash; \
			 cd ~/rmf/src/soss/soss; \
			 ./soss.py ../configs/android_ros2_ros1.yaml; \
			 exec bash'";

echo "Initialize SOSS: ROS1 -> ROS2 for /ldr_value"
terminator --new-tab -x "sshpass -p '$1' ssh -t '$2' \
			'source ~/rmf/build/soss/setup.bash; \
			 cd ~/rmf/src/soss/soss; \
			 ./soss.py ../configs/android_ros1_ros2.yaml; \
			 exec bash'";
