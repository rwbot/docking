#include <docking/SegmentLine.h>
using namespace docking;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//constructor:
template <typename PointT>
SegmentLine<PointT>::SegmentLine() {}

//de-constructor:
template <typename PointT>
SegmentLine<PointT>::~SegmentLine() {}

template <typename PointT>

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
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a line model for the given dataset." << std::endl;
    }
    //
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "Line segmentation took " << elapsedTime.count() << " microseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> line; // = <coefficients, inliers>;
    return line;
}
