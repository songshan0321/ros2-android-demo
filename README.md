# ros2-android-demo

<img src="pics/ros2-android.jpg" width="300"><img src="pics/GUI.jpg" width="300">

### Block Diagram

<img src="pics/diagram.png">



### -Installation-

### Method 1: 

On **robot's computer**, install ros2-android-demo & run [demo_install.sh](scripts/demo_install.sh):

```bash
cd ~/catkin_ws/src
git clone https://github.com/songshan0321/ros2-android-demo.git
cd ~/catkin_ws/src/ros2-android-demo/scripts
./demo_install.sh
```

On **server computer**, create a symlink of [bringup.sh](scripts/bringup.sh) at root directory:

```bash
git clone https://github.com/songshan0321/ros2-android-demo.git
ln -s ~/github/ros2-android-demo/scripts/bringup.sh ~/bringup.sh
```

On **server computer**, git clone virtual-joystick library:

```bash
cd ~/github/ros2-android-demo/app/base_controller
mkdir libs
cd libs && git clone https://github.com/controlwear/virtual-joystick-android.git
```

------

### Method 2:

On **robot's computer**, download and install ros2-android-demo:

```bash
cd ~/catkin_ws/src
git clone https://github.com/songshan0321/ros2-android-demo.git
# install serial package as dependencies
git clone https://github.com/wjwwood/serial.git
# Start catkin make
cd ~/catkin_ws && catkin_make
# Copy 2 yaml files into soss configs folder
cp src/ros2-android-demo/configs/android_ros2_ros1.yaml ~/rmf/src/soss/configs/android_ros2_ros1.yaml
cp src/ros2-android-demo/configs/android_ros1_ros2.yaml ~/rmf/src/soss/configs/android_ros1_ros2.yaml
```

Manually add required msgs into **soss_msgs** yaml file, then rebuild SOSS:

```bash
echo -e " - std_msgs/Int16 \n - geometry_msgs/Vector3 \n - geometry_msgs/Twist" >> ~/rmf/src/rmf/soss/interfaces.yaml
cat ~/rmf/src/rmf/soss/interfaces.yaml
cd ~/rmf/src/rmf/scripts && ./generate_soss.sh
# Added additional msgs to SOSS successfully!
```

Install PlatformIO and upload compiled Arduino base code:

```bash
pip install -U platformio
cd ~/catkin_ws/src/ros2-android-demo/platformio/car_base/
platformio run -e uno --target upload
```

On **server computer**, create a symlink of [bringup.sh](scripts/bringup.sh) at root directory:

```bash
git clone https://github.com/songshan0321/ros2-android-demo.git
ln -s ~/github/ros2-android-demo/scripts/bringup.sh ~/bringup.sh
```

---

### Setup ros2-android-app project for compile

Follow instruction from [ros2-android-app](https://github.com/songshan0321/ros2-android-app) to compile the project for further development.

---

## Run Demo

### Method 1: 

On **server computer**, bring up robot:

Note: input user@host and password of robot's computer accordingly.

```bash
./bringup.sh
```

---

### Method 2:

### Step1: Launch remote robot car

On **robot's computer**, 

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

Start control your robot by Android App from mobile. Have fun!



### Step5: (Optional) Echo msgs

On **server computer**, echo /cmd_vel & /ldr_value topic:

```bash
. ~/rmf/build/ros2/install/setup.bash
ros2 topic echo /cmd_vel
ros2 topic echo /ldr_value
```



### Future Development



