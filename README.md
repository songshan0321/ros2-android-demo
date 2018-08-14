## ROS2_Android_App

### Installation

Download and install ROS2_Android_App on **robotcar's computer**,

```bash
cd ~/catkin_ws/src
git clone https://github.com/songshan0321/ROS2_Android_App.git
# install serial package for robot if you do not have
git clone https://github.com/wjwwood/serial.git
# Start catkin make
cd ~/catkin_ws && catkin_make
```



Download and install soss_demo on **server computer**,

```bash
cd ~/ros2_android_ws/src
git clone https://github.com/songshan0321/ROS2_Android_App.git
# Copy a yaml file into soss configs folder
cp ROS2_Android_App/configs/android_ros2_ros1.yaml ~/rmf/src/soss/configs/android_ros2_ros1.yaml
```



## Run Demo

### Step1: Launch remote robot car

On **robot's computer** (10.10.1.206), 

SSH into robot's computer:

```bash
ssh demo@10.10.7.186
```

Run subscriber on robot:

```bash
. ~/catkin_ws/devel/setup.bash
sudo chmod 777 /dev/ttyACM0
roslaunch move_robot_android car.launch
```



### Step2: translate ROS 2 messages to ROS 1 messages

On another terminal, source the SOSS libraries and start a SOSS instance:

```bash
. ~/rmf/build/soss/setup.bash
cd ~/rmf/src/soss/soss
./soss.py ../configs/android_ros2_ros1.yaml
```



### Step3: Open RMFTool Android App on your phone

Start control robotcar. Have fun!



### Step4: (Optional) Echo msg on RMFChatter

On **server computer**, echo RMFChatter topic:

```bash
. ~/rmf/build/ros2/install/setup.bash
ros2 topic echo /RMFChatter
```

