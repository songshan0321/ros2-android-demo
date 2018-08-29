#!/bin/bash

echo "To bring up robot, please enter IP and password."
read -p 'user@host: ' hostvar
read -sp 'Password: ' passvar

terminator --new-tab -x "sshpass -p '$passvar' ssh -t '$hostvar' \
			'source ~/catkin_ws/devel/setup.bash; \
			 echo '$passvar' | sudo -S chmod 777 /dev/ttyACM0; \
			 roslaunch remote_robot_android car.launch; \
			 bash'";

echo "Initialize SOSS: ROS2 -> ROS1 for /cmd_vel"
terminator --new-tab -x "sshpass -p '$passvar' ssh -t '$hostvar' \
			'source ~/rmf/build/soss/setup.bash; \
			 cd ~/rmf/src/soss/soss; \
			 ./soss.py ../configs/android_ros2_ros1.yaml; \
			 bash'";

echo "Initialize SOSS: ROS1 -> ROS2 for /ldr_value"
terminator --new-tab -x "sshpass -p '$passvar' ssh -t '$hostvar' \
			'source ~/rmf/build/soss/setup.bash; \
			 cd ~/rmf/src/soss/soss; \
			 ./soss.py ../configs/android_ros1_ros2.yaml; \
			 bash'";
