#ifndef SEGMENTLINE_H_
#define SEGMENTLINE_H_

#include <pcl/io/io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
// typedef pcl_ros::SACSegmentation SACSegmentation ;
namespace docking{

template <typename PointT>
class SegmentLine{
public:
    SegmentLine();
    ~SegmentLine();

    std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> RansacLine(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
};
}

#endif
