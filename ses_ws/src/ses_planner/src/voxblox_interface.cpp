#include "ros/ros.h"
#include <erc_map_manager.h>


VoxbloxInterface::VoxbloxInterface(ros::NodeHandle &nh, ros::NodeHandle &nh_private): nh_(nh), nh_private_(nh_private)
{
  lineSub = nh_.subscribe("line_status", 1000, &VoxbloxInterface::checkLineStatus, this);
  mapdistSub = nh_.subscribe("map_status", 1000, &VoxbloxInterface::checkMapDistance, this);
  boxSub = nh_.subscribe("box_status", 1000, &VoxbloxInterface::checkBoxStatus, this);
  VoxbloxMapManager* map_manager_ = new VoxbloxMapManager(nh_, nh_private_);
  
  std::cout << "VOXBLOX INTERFACE INITIALISED, SUBSCRIBERS INITIALISED"<<std::endl;
}



void VoxbloxInterface::checkLineStatus(const geometry_msgs::PoseStamped& msg )
{
  float x1,y1,z1,x2,y2,z2, bx, by, bz;
  std::cout << "Enter start point in x1,y1,z1"<<std::endl;
  std::cin >> x1 >> y1 >>z1;
  std::cout << "Enter end point in x2, y2, z2"<< std::endl;
  std::cin >> x2 >> y2 >>z2;
  std::cout << "Enter box size in x,y,z"<<std::endl;
  std::cin >> bx >> by >>bz;

  Eigen::Vector3d start(x1,y1,z1);
  Eigen::Vector3d end(x2,y2,z2);
  Eigen::Vector3d box_size(bx,by,bz);
  int result = int(this->map_manager_->getPathStatus(start, end, box_size, true));
  if (result == 0)
  {
	  std::cout << "\n\nLINE UNKNOWN"<<std::endl;
  }
  else if (result == 1)
  {
	  std::cout << "\n\nLINE FREE"<<std::endl;
  }
  else 
  {
	  std::cout << "\n\nLINE OCCUPIED"<<std::endl;
  }

}

void VoxbloxInterface::checkBoxStatus(const geometry_msgs::PoseStamped& msg )
{
  float x1,y1,z1, bx, by, bz;
  std::cout << "Enter start point in x1,y1,z1"<<std::endl;
  std::cin >> x1 >> y1 >>z1;
  std::cout << "Enter box size in x,y,z"<<std::endl;
  std::cin >> bx >> by >>bz;

  Eigen::Vector3d start(x1,y1,z1);
  Eigen::Vector3d box_size(bx,by,bz);
  int result = int(this->map_manager_->getBoxStatus(start, box_size, true));
  if (result == 0)
  {
	  std::cout << "\n\nBOX UNKNOWN"<<std::endl;
  }
  else if (result == 1)
  {
	  std::cout << "\n\nBOX FREE"<<std::endl;
  }
  else 
  {
	  std::cout << "\n\nBOX OCCUPIED"<<std::endl;
  }

}

void VoxbloxInterface::checkMapDistance(const geometry_msgs::PoseStamped& msg )
{
  float x1,y1,z1;
  std::cout << "Enter start point in x1,y1,z1"<<std::endl;
  std::cin >> x1 >> y1 >>z1;

  Eigen::Vector3d start(x1,y1,z1);
  double result = double(this->map_manager_->getMapDistance(start));
  std::cout << "\n\nDistance of map from point is :- "<<result <<std::endl;
}