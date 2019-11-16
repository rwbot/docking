//#ifndef SEGMENTLINE_IMPL_H_
//#define SEGMENTLINE_IMPL_H_
////#include <docking/SegmentLine.h>

//#include <pcl/io/io.h>
//#include <pcl/common/common.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/common/transforms.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_line.h>
//#include <pcl/impl/instantiate.hpp>
//#include <iostream>
//#include <string>
//#include <vector>
//#include <ctime>
//#include <chrono>
//// typedef pcl_ros::SACSegmentation SACSegmentation ;
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

////template <typename PointT>
////docking<>

//template <typename PointT>
//std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> RansacLine(typename pcl::PointCloud<PointT>::Ptr cloud,
//                                                                          int maxIterations,
//                                                                          float distanceThreshold)
//{
////    auto startTime = std::chrono::steady_clock::now();
////    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
////    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
////    //Create the segmentation object
////    pcl::SACSegmentation<PointT> seg;
////    seg.setOptimizeCoefficients(true);
////    seg.setModelType(pcl::SACMODEL_LINE);
////    seg.setMethodType(pcl::SAC_RANSAC);
////    seg.setMaxIterations(maxIterations);
////    seg.setDistanceThreshold(distanceThreshold);
////    //Segment the largest planar component from the input cloud
////    seg.setInputCloud(cloud);
////    seg.segment(*inliers, *coefficients);
////    if (inliers->indices.size() == 0)
////    {
////        std::cout << "Could not estimate a line model for the given dataset." << std::endl;
////    }
////    //
////    auto endTime = std::chrono::steady_clock::now();
////    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
////    std::cout << "Line segmentation took " << elapsedTime.count() << " microseconds" << std::endl;

//    //          std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
////    std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> line(coefficients, inliers);
//    std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> line(cloud, cloud);
//    return line;
//}

////#define PCL_INSTANTIATE_SegmentLine(T) template class PCL_EXPORTS pcl::SegmentLine<T>;

//#endif /*"SEGMENTLINE_IMPL_H_"*/
