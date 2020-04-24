#include "ros/ros.h"
#include <erc_map_manager.h>
#include <std_msgs/String.h>



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


//   ros::AsyncSpinner spinner(2);
//   spinner.start();
//   ros::waitForShutdown();
ros::spin();
return 0;
}
