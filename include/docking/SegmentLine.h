#ifndef SEGMENTLINE_H_
#define SEGMENTLINE_H_

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
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
// typedef pcl_ros::SACSegmentation SACSegmentation ;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//namespace docking{

  template<typename PointT>
  class SegmentLine : public pcl::PCLBase<PointT>
  {

    public:
      using pcl::PCLBase<PointT>::input_;
      SegmentLine(){}
      ~SegmentLine(){}

 ///////////////// BEGIN RANSAC LINE /////////////////
      std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> RansacLine(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
    {
      auto startTime = std::chrono::steady_clock::now();
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      //Create the segmentation object
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_LINE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(maxIterations);
      seg.setDistanceThreshold(distanceThreshold);
      //Segment the largest planar component from the input cloud
//      seg.setInputCloud(cloud);
      seg.setInputCloud(input_);
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0) { std::cout << "Could not estimate a line model for the given dataset." << std::endl; }
      //
      auto endTime = std::chrono::steady_clock::now();
      auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
      std::cout << "RANSAC took " << elapsedTime.count() << " microseconds" << std::endl;

//                std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
      std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> line(coefficients, inliers);
        return line;
      }
///////////////// END RANSAC LINE /////////////////
  };

//}

//template std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> docking::SegmentLine<pcl::PointXYZ>::RansacLine();
//<pcl::PointCloud<pcl::PointXYZ>::Ptr, int, float>(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceThreshold);

//#include <docking/impl/SegmentLine.hpp>


//#define PCL_INSTANTIATE_SegmentLine(T) template class PCL_EXPORTS pcl::SegmentLine<T>;
  template class SegmentLine<pcl::PointXYZI>;
  template class SegmentLine<pcl::PointXYZ>;
  template class SegmentLine<pcl::PointXYZRGB>;
  template class SegmentLine<pcl::PointXYZRGBA>;
//  template class SegmentLine<pcl::PointCloud<pcl::PointXYZ>>;

#endif /*"SEGMENTLINE_H_"*/
