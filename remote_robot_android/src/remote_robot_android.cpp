#include "ros/ros.h"
#include <serial/serial.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include <string>
#include <iostream>
#include <sstream>

serial::Serial ser;

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Read in msg
  double linearVelX = msg->linear.x;
  double angVelZ = msg->angular.z;
  std::string motor_msg;
  std::stringstream ss_linear;
  std::stringstream ss_angular;

  // Serial write
  ss_linear << linearVelX;
  ss_angular << angVelZ;
  motor_msg = ss_linear.str() + "," +ss_angular.str() + "\n";
  ser.write(motor_msg);
  ROS_INFO_STREAM(motor_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, chatterCallback);
  ros::Publisher pub = n.advertise<std_msgs::Int16>("ldr_value", 1000);

  // Open serial port 
  try
  {
      ser.setPort("/dev/ttyACM0");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
  }
  catch (serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }

  if(ser.isOpen()){
      ROS_INFO_STREAM("Serial Port initialized");
  }else{
      return -1;
  }

    ros::Rate loop_rate(50);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            // ROS_INFO_STREAM("Reading from serial port");
            std_msgs::Int16 value;
            std::string strValue;
            std::stringstream ss;

            strValue = ser.read(ser.available());
            ss.clear();
            ss.str("");
            ss.str(strValue);
            ss >> value.data;

            if (value.data != 0){ // ignore '0' data
              ROS_INFO_STREAM("Sensor value: " << value.data);
              pub.publish(value);
            }
        }
        loop_rate.sleep();
    }

  return 0;
}
