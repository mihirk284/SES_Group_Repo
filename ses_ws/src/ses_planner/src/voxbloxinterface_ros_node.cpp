#include "ros/ros.h"
#include <erc_map_manager.h>
#include <std_msgs/String.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void psCallback(const geometry_msgs::PoseStamped& msg)
{
  std::cout << msg.pose.position.x <<std::endl;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "sesplanner_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");


  VoxbloxInterface vbint(nh, nh_private);
  ros::Subscriber randomSub = nh.subscribe("testsub", 1000, chatterCallback);
  ros::Subscriber psSub = nh.subscribe("pssub", 1000, psCallback);
  std::cout << "VBINT intialised"<<std::endl;


//   ros::AsyncSpinner spinner(2);
//   spinner.start();
//   ros::waitForShutdown();
ros::spin();
return 0;
}
