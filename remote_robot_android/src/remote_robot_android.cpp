#include "ros/ros.h"
#include <serial/serial.h>
#include "std_msgs/String.h"
#include <string>

serial::Serial ser;

std::string data;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  // Read in msg
  data = msg->data;
  ROS_INFO("I heard: [%s]", data);

  // Serial write

  // Forward
  if (strcmp("stop",data)){
    ROS_INFO("Writing 0 to serial port");
    ser.write("0");
  }
  else if (strcmp("right",data)){
    ROS_INFO("Writing 1 to serial port");
    ser.write("1");
  }
  else if (strcmp("left",data)){
    ROS_INFO("Writing 2 to serial port");
    ser.write("2");
  }
  else if (strcmp("forward",data)){
    ROS_INFO("Writing 3 to serial port");
    ser.write("3");
  }
  else if (strcmp("backward",data)){
    ROS_INFO("Writing 4 to serial port");
    ser.write("4");
  }
  else{
    ROS_INFO("Invalid direction msg.");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "remote_robot_android");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("RMFChatter", 1000, chatterCallback);
  ros::Publisher read_pub = n.advertise<std_msgs::String>("read", 1000);

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

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();
    }

  return 0;
}
