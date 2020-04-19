#include <voxblox_ros/esdf_server.h>
#include <eigen3/Eigen/Dense>
#include <voxblox_ros/esdf_server.h>
#include <voxblox/utils/planning_utils.h>
#include <voxblox/utils/planning_utils_inl.h>
#include <voxblox_ros/transformer.h>
#include <voxblox_ros/ros_params.h>
#include <iostream>

#include "geometry_msgs/PoseStamped.h"
#include "ses_planner/checkBoxes.h"

class VoxbloxMapManager {
 public:
  enum VoxelStatus{ kUnknown =0, kOccupied, kFree};
  VoxbloxMapManager(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);
  virtual ~VoxbloxMapManager() {}
  double getMapDistance(const Eigen::Vector3d& position) const;
  VoxelStatus getBoxStatus(const Eigen::Vector3d& center,const Eigen::Vector3d& size,bool stop_at_unknown_voxel) const;
  VoxelStatus getPathStatus(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel) const;
  VoxelStatus getBoxStatusInVoxels(const voxblox::LongIndex& box_center, const voxblox::AnyIndex& box_voxels, bool stop_at_unknown_voxel) const;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  voxblox::EsdfServer voxblox_server_;
  voxblox::EsdfVoxel voxblox_voxel_;
  voxblox::Layer<voxblox::EsdfVoxel>* sdf_layer_;
  float occupancy_distance_voxelsize_factor_;
  bool checkUnknownStatus(const voxblox::EsdfVoxel* voxel) const;

  // Map!
  
};

class VoxbloxInterface
{
  public:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber lineSub, mapdistSub, boxSub;
	VoxbloxInterface(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  VoxbloxMapManager* map_manager_;
	void checkLineStatus(const geometry_msgs::PoseStamped& msg);
	void checkBoxStatus(const geometry_msgs::PoseStamped& msg);
	void checkMapDistance(const geometry_msgs::PoseStamped& msg);
  bool checkBoxesService(ses_planner::checkBoxes::Request& req, ses_planner::checkBoxes::Response& res);
  ros::ServiceServer service;
};
