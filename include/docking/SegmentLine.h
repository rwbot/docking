#ifndef SEGMENTLINE_H_
#define SEGMENTLINE_H_
#include <ros/ros.h>
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
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//namespace docking{

  template<typename PointT>
  class SegmentLine
  {

    public:
      SegmentLine(){}
      ~SegmentLine(){}

      typename pcl::PointCloud<PointT>::Ptr input;
      typename pcl::PointCloud<PointT>::Ptr output;

      void setInputCloud(typename pcl::PointCloud<PointT>::Ptr in)
      {
        input = in;
      }

      void setParams(typename pcl::PointCloud<PointT>::Ptr in)
      {
        input = in;
      }

      ///////////////// BEGIN RANSAC LINE /////////////////
      /// \brief RansacLine
      /// \param inputCloudPtr
      /// \param maxIterations
      /// \param distanceThreshold
      /// \return lines
      ///
     std::vector<typename pcl::PointCloud<PointT>::Ptr> RansacLine(typename pcl::PointCloud<PointT>::Ptr inputCloudPtr, int maxIterations, float distanceThreshold)
//     std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> RansacLine(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
    {
//      auto startTime = std::chrono::steady_clock::now();
      int minInliers = 10;
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      //Create the segmentation object
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_LINE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(maxIterations);
      seg.setDistanceThreshold(distanceThreshold);

      std::vector<typename pcl::PointCloud<PointT>::Ptr > lines;
      std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
      typename pcl::PointCloud<PointT>::Ptr inCloudPtr, outCloudPtr;
      inCloudPtr = inputCloudPtr;
      int i = 1;
      while(inCloudPtr->size() != 0){
        std::cout <<  std::endl <<"Iteration # " << i++ << std::endl;
        if (i > 100) { break; }
        //Segment the largest line component from the input cloud
        seg.setInputCloud(inCloudPtr);
        seg.segment(*inliers, *coefficients);

        segResult = SeparateClouds(inliers, inCloudPtr);

        typename pcl::PointCloud<PointT>::Ptr line (new pcl::PointCloud<PointT>);
        line = segResult.first;
        line->width = inliers->indices.size();
        line->height = 1;
        line->is_dense = true;
        lines.push_back(line);
        inCloudPtr = segResult.second;
        if (inliers->indices.size() <= minInliers) {
          std::cout << std::endl << "Less than " << minInliers << " inliers left" << std::endl;
          break;
        }
      }

      std::cout << "RANSAC: " << lines.size() << " lines found" << std::endl;
//      auto endTime = std::chrono::steady_clock::now();
//      auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
//      std::cout << "RANSAC took " << elapsedTime.count() << " microseconds" << std::endl;

//      std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> line(coefficients, inliers);
      return lines;
      }
///////////////// END RANSAC LINE /////////////////

     ///////////////// BEGIN SEPARATE CLOUDS /////////////////
     /// \brief SeparateClouds
     /// \param inliers
     /// \param cloud
     /// \return
     ///
      std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
      {
          typename pcl::PointCloud<PointT>::Ptr inCloud (new pcl::PointCloud<PointT>());
          typename pcl::PointCloud<PointT>::Ptr outCloud (new pcl::PointCloud<PointT>());

          // Obtain the inlier point cloud by copying only inlier indices to the planeCloud
          // for(int index : inliers->indices)
          //     inCloud->points.push_back(cloud->points[index]);
          // ****** ^ does the same as below for the inliers, cuz we already have the inlier indices******
          // Create extract object
          pcl::ExtractIndices<PointT> extract;
          extract.setInputCloud (cloud);
          extract.setIndices (inliers);

          // Get plane cloud (inliers)
          extract.setNegative (false);    // Extract the inliers, not outliers
          extract.filter (*inCloud);   //Output cloud

          // Get obstacle cloud (outliers)
          extract.setNegative (true);     // Extract the outliers, not inlierss
          extract.filter (*outCloud);    //Output cloud
          std::cerr << "Inliers: " << inCloud->width * inCloud->height << " points " << " Outliers: " << outCloud->width * outCloud->height << " points" << std::endl;

          std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(inCloud, outCloud);
          return segResult;
      }
///////////////// END SEPARATE CLOUDS /////////////////

      ///////////////// BEGIN CLUSTER LINES /////////////////
      /// \brief ClusterLines
      /// \param cloud
      /// \param clusterTolerance
      /// \param minSize
      /// \param maxSize
      /// \return
      ///
      typename pcl::PointCloud<PointT>::Ptr ClusterLines(typename pcl::PointCloud<PointT>::Ptr cloud, double clusterTolerance, int minSize, int maxSize)
      {
        typename pcl::PointCloud<PointT>::Ptr inCloud (new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr outCloud (new pcl::PointCloud<PointT>());
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (clusterTolerance);
        ec.setMinClusterSize (minSize);
        ec.setMaxClusterSize (maxSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract(cluster_indices);

        std::vector<typename pcl::PointCloud<PointT>::Ptr > clusters;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back(cloud->points[*pit]);

          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;
          std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
          // Add current cluster to list of clusters
          clusters.push_back(cloud_cluster);
        }
      }
///////////////// END CLUSTER LINES /////////////////

      ///////////////// BEGIN VOXEL GRID /////////////////
      /// \brief VoxelGrid
      /// \param inCloudPtr
      /// \param leafSize
      /// \return
      ///

      typename pcl::PointCloud<PointT>::Ptr VoxelGrid(typename pcl::PointCloud<PointT>::Ptr inCloudPtr, float leafSize)
      {
        typename pcl::PointCloud<PointT>::Ptr filteredCloudPtr (new pcl::PointCloud<PointT>());
        // Perform the actual filtering
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(inCloudPtr);
        voxel.setLeafSize(leafSize, leafSize, leafSize);
        voxel.filter(*filteredCloudPtr);
      }
///////////////// END VOXEL GRID /////////////////

  };

//}


//#include <docking/impl/SegmentLine.hpp>
//#define PCL_INSTANTIATE_SegmentLine(T) template class PCL_EXPORTS pcl::SegmentLine<T>;

  template class SegmentLine<pcl::PointXYZI>;
  template class SegmentLine<pcl::PointXYZ>;
  template class SegmentLine<pcl::PointXYZRGB>;
  template class SegmentLine<pcl::PointXYZRGBA>;

#endif /*"SEGMENTLINE_H_"*/
