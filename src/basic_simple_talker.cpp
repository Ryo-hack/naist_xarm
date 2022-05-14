#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_simple_talker");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 10);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world!";
    ROS_INFO("publish: %s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}