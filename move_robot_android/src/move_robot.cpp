#include "ros/ros.h"
#include <serial/serial.h>
#include "std_msgs/String.h"

serial::Serial ser;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  // Read in msg
  const char* data = msg->data.c_str();
  ROS_INFO("I heard: [%s]", data);
  // Serial write if button is pressed
  if (strcmp("left",data) == 0){
    ROS_INFO("Writing 0 to serial port");
    ser.write("0");
  }
  if (strcmp("right",data) == 0){
    ROS_INFO("Writing 1 to serial port");
    ser.write("1");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_robot_android");
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
