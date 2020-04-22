#include <erc_map_manager.h>


VoxbloxMapManager::VoxbloxMapManager(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private): voxblox_server_(nh, nh_private)
                                                                                                  , occupancy_distance_voxelsize_factor_(1.0F)
{
  voxblox::EsdfVoxel voxblox_voxel_;
  sdf_layer_ = voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr();

}


double VoxbloxMapManager::getMapDistance(
    const Eigen::Vector3d& position) const {
  if (!voxblox_server_.getEsdfMapPtr()) {
    return 0.0;
  }
  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,&distance)) {
    return 0.0;
  }
  return distance;
}

VoxbloxMapManager::VoxelStatus VoxbloxMapManager::getBoxStatus(const Eigen::Vector3d& center,
                                    const Eigen::Vector3d& size,
                                    bool stop_at_unknown_voxel) const
{
  std::cout << "GETTING BOX STATUS "<<std::endl;
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1 / voxel_size;
  voxblox::LongIndex center_voxel_index = voxblox::getGridIndexFromPoint<voxblox::LongIndex>(center.cast<voxblox::FloatingPoint>(), voxel_size_inv);

  voxblox::AnyIndex box_voxels(std::ceil(size.x() * voxel_size_inv),
                               std::ceil(size.y() * voxel_size_inv),
                               std::ceil(size.z() * voxel_size_inv));
  return getBoxStatusInVoxels(center_voxel_index, box_voxels, stop_at_unknown_voxel);
}

VoxbloxMapManager::VoxelStatus VoxbloxMapManager::getBoxStatusInVoxels(const voxblox::LongIndex& box_center,
                                                                       const voxblox::AnyIndex& box_voxels,
                                                                       bool stop_at_unknown_voxel) const{
VoxelStatus current_status = VoxelStatus::kFree;
const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;
voxblox::LongIndexElement x_beg = box_center.x() - box_voxels.x()/2;
voxblox::LongIndexElement x_end = box_center.x() + box_voxels.x()/2;
voxblox::LongIndexElement y_beg = box_center.y() - box_voxels.y()/2;
voxblox::LongIndexElement y_end = box_center.y() + box_voxels.y()/2;
voxblox::LongIndexElement z_beg = box_center.z() - box_voxels.z()/2;
voxblox::LongIndexElement z_end = box_center.z() + box_voxels.z()/2;
for(voxblox::LongIndexElement i = x_beg; i <= x_end; i++)
{
  for(voxblox::LongIndexElement j = y_beg; j <= y_end; j++)
  {
    for(voxblox::LongIndexElement k = z_beg; k <= z_end; k++)
    {
      voxblox::LongIndex voxel_index(i, j, k);
      voxblox::EsdfVoxel* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(voxel_index);
      if (checkUnknownStatus(voxel))
      {
        std::cout << "Voxel Unknown"<<std::endl;
        if (stop_at_unknown_voxel)
        {
          return VoxelStatus::kUnknown;
        }
        current_status = VoxelStatus::kUnknown;
      }
      else if (voxel->distance <= distance_thres)
      {
        std::cout << "Voxel Distance less than threshold, setting as occupied"<<std::endl;
        return VoxelStatus::kOccupied;
      }

    }
  }
}
//std::cout << "BOX STATUS IS CURRENT STATUS, SEEMS FREE"<<std::endl;
return current_status;
}


bool VoxbloxMapManager::checkUnknownStatus(const voxblox::EsdfVoxel* voxel) const {
  if (voxel == nullptr ||  !voxel->observed) {     //voxel->weight < 1e-6) {
    std::cout << "Voxel Unobserved"<<std::endl;
    return true;
  }
  std::cout << "Voxel Observed"<<std::endl;
  return false;
}


VoxbloxMapManager::VoxelStatus VoxbloxMapManager::getPathStatus(const Eigen::Vector3d& start,
                                                                const Eigen::Vector3d& end,
                                                                const Eigen::Vector3d& box_size,
                                                                bool stop_at_unknown_voxel) const {
  // Cast ray along the center to make sure we don't miss anything.
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  const voxblox::Point start_scaled =
      start.cast<voxblox::FloatingPoint>() * voxel_size_inv;
  const voxblox::Point end_scaled =
      end.cast<voxblox::FloatingPoint>() * voxel_size_inv;

  voxblox::LongIndexVector global_voxel_indices;
  voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

  // Get the bounding box size in terms of voxels.
  voxblox::AnyIndex box_voxels(
      std::ceil(box_size.x() * voxel_size_inv),
      std::ceil(box_size.y() * voxel_size_inv),
      std::ceil(box_size.z() * voxel_size_inv));

  // Iterate over the ray.
  VoxelStatus current_status = VoxelStatus::kFree;
  for (const voxblox::GlobalIndex& global_index : global_voxel_indices) {
    VoxelStatus box_status = getBoxStatusInVoxels(
        global_index, box_voxels, stop_at_unknown_voxel);
    if (box_status == VoxelStatus::kOccupied) {
      return box_status;
    }
    if (stop_at_unknown_voxel && box_status == VoxelStatus::kUnknown) {
      return box_status;
    }
    current_status = box_status;
  }
  return current_status;
}
