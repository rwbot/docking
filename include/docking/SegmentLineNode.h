#ifndef SEGMENTLINENODE_H_
#define SEGMENTLINENODE_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <pcl/io/io.h>
#include <pcl/common/common.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

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

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace docking{

  template<typename PointT>
  class SegmentLineNode
  {

    public:
      SegmentLineNode(ros::NodeHandle nh) : nh_(nh)
      {
        pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/dockLines", 1);
        sub_  = nh_.subscribe("/cloud", 1, &SegmentLineNode::cloudCallback, this);
      }
      ~SegmentLineNode(){}

      ros::Publisher pub_;
      ros::Subscriber sub_;
      ros::NodeHandle nh_;

      void startPub(){
        pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/dockLines", 1);
      }

      void startSub(){
        sub_  = nh_.subscribe("/dockLines", 1, &SegmentLineNode::cloudCallback, this);
      }

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



      void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
      //void cloud_cb(const pcl::PointCloud<PointT>::ConstPtr &msg)
      {
        ROS_INFO_STREAM("Callback Called: ");
          // Container for original & filtered data
          /*
          * CONVERT POINTCLOUD ROS->PCL
          */
          pcl::PointCloud<PointT> cloudPCL;
          pcl::fromROSMsg (*msg, cloudPCL);
          typename pcl::PointCloud<PointT>::Ptr cloudPCLPtr (new pcl::PointCloud<PointT> (cloudPCL));


          /* ========================================
           * CONVERT POINTCLOUD PCL->ROS
           * PUBLISH CLOUD
           * ========================================*/
          sensor_msgs::PointCloud2::Ptr cloudROSPtr (new sensor_msgs::PointCloud2);
          pcl::toROSMsg(*cloudPCLPtr, *cloudROSPtr);
      //    cloudROSPtr->header.frame_id=world_frame;
      //    cloudROSPtr->header.stamp=ros::Time::now();
          pub_.publish(cloudROSPtr);
      }

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

  template class SegmentLineNode<pcl::PointXYZ>;
  template class SegmentLineNode<pcl::PointXYZI>;
  template class SegmentLineNode<pcl::PointXYZRGB>;
  template class SegmentLineNode<pcl::PointXYZRGBA>;

}// END NAMESPACE DOCKING


//#include <docking/impl/SegmentLineNode.hpp>
//#define PCL_INSTANTIATE_SegmentLineNode(T) template class PCL_EXPORTS pcl::SegmentLineNode<T>;



#endif /*"SEGMENTLINENODE_H_"*/
