#include <pcl/io/io.h>
#include <pcl/common/common.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <docking/SegmentLine.h>

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr VoxelGrid(typename pcl::PointCloud<PointT>::Ptr inCloudPtr, float leafSize)
{
  typename pcl::PointCloud<PointT>::Ptr filteredCloudPtr (new pcl::PointCloud<PointT>());
  // Perform the actual filtering
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(inCloudPtr);
  voxel.setLeafSize(leafSize, leafSize, leafSize);
  voxel.filter(*filteredCloudPtr);
}

