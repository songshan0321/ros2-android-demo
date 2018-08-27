#!/bin/bash

# Install ros2-android-demo (Run this on robot's computer)
echo "Installation for 'ros2-android-demo' started!"

echo "Install serial package as dependencies"
cd ~/catkin_ws/src
git clone https://github.com/wjwwood/serial.git
echo "Start catkin make"
cd ~/catkin_ws && catkin_make
echo "Copy 2 yaml files into soss configs folder"
cp src/ros2-android-demo/configs/android_ros2_ros1.yaml ~/rmf/src/soss/configs/android_ros2_ros1.yaml
cp src/ros2-android-demo/configs/android_ros1_ros2.yaml ~/rmf/src/soss/configs/android_ros1_ros2.yaml

echo -e " - std_msgs/Int16 \n - geometry_msgs/Vector3 \n - geometry_msgs/Twist" >> ~/rmf/src/rmf/soss/interfaces.yaml
cat ~/rmf/src/rmf/soss/interfaces.yaml
echo "Rebuild SOSS"
cd ~/rmf/src/rmf/scripts && ./generate_soss.sh
echo "Added additional msgs to SOSS successfully!"
# Install PlatformIO
if ! command -v platformio > /dev/null; then
   echo -e "PlatformIO is not installed! Install? (y/n) \c"
   read REPLY
   if [ $REPLY = "y" ]
   then
      pip install -U platformio
   elif [ $REPLY = "n" ]
   then
      echo "Ok! Remember to install PlatformIO yourself."
   fi
fi
echo "Upload car base to Arduino"
cd ~/catkin_ws/src/ros2-android-demo/platformio/car_base/
platformio run -e uno --target upload
echo "Installed 'ros2-android-demo' successfully"

