# ros2-android-demo

### Installation

On **robotcar's computer**, download and install ros2-android-demo:

```bash
cd ~/catkin_ws/src
git clone https://github.com/songshan0321/ros2-android-demo.git
# install serial package as dependencies
git clone https://github.com/wjwwood/serial.git
# Start catkin make
cd ~/catkin_ws && catkin_make
# Copy 2 yaml files into soss configs folder
cp ROS2_Android_App/configs/android_ros2_ros1.yaml ~/rmf/src/soss/configs/android_ros2_ros1.yaml
cp ROS2_Android_App/configs/android_ros1_ros2.yaml ~/rmf/src/soss/configs/android_ros1_ros2.yaml
```

Manually add required msgs into **soss_msgs** yaml file, then rebuild SOSS:

```bash
echo " - std_msgs/Int16" >> ~/rmf/src/rmf/soss/interfaces.yaml
cat ~/rmf/src/rmf/soss/interfaces.yaml
cd ~/rmf/src/rmf/scripts && ./generate_soss.sh
# Added std_msgs/Int16 to SOSS successfully!

echo " - geometry_msgs/Vector3" >> ~/rmf/src/rmf/soss/interfaces.yaml
echo " - geometry_msgs/Twist" >> ~/rmf/src/rmf/soss/interfaces.yaml
cat ~/rmf/src/rmf/soss/interfaces.yaml
cd ~/rmf/src/rmf/scripts && ./generate_soss.sh
# Added geometry_msgs/Twist to SOSS successfully!
```

On **server computer**, create a symlink of 'bringup.sh' at home page:

```bash
ln -s ~/github/ros2-android-demo/bringup.sh ~/bringup.sh
```



## Run Demo

### ~Method 1~

On **server computer**, 

```bash
./bringup [password] [user@host]
```

For example:

```bash
./bringup.sh password1234 david@192.168.1.1
```

### ~Method 2~

### Step1: Launch remote robot car

On **robot's computer** (10.10.1.206), 

SSH into robot's computer:

```bash
ssh demo@10.10.7.186
```

Run subscriber on robot:

```bash
. ~/catkin_ws/devel/setup.bash
echo rmf1234 | sudo -S chmod 777 /dev/ttyACM0
roslaunch remote_robot_android car.launch
```



### Step2: translate ROS 2 to ROS 1 messages

On another terminal, source the SOSS libraries and start a SOSS instance:

```bash
. ~/rmf/build/soss/setup.bash
cd ~/rmf/src/soss/soss
./soss.py ../configs/android_ros2_ros1.yaml
```

### Step3: translate ROS 1 to ROS 2 messages

On another terminal, source the SOSS libraries and start a SOSS instance:

```bash
. ~/rmf/build/soss/setup.bash
cd ~/rmf/src/soss/soss
./soss.py ../configs/android_ros1_ros2.yaml
```



### Step4: Open ROS2 Controller App on your phone

Start control your robot. Have fun!



### Step5: (Optional) Echo msgs

On **server computer**, echo /cmd_vel & /ldr_value topic:

```bash
. ~/rmf/build/ros2/install/setup.bash
ros2 topic echo /cmd_vel
ros2 topic echo /ldr_value
```

