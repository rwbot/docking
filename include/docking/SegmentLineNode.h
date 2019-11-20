#ifndef SEGMENTLINENODE_H_
#define SEGMENTLINENODE_H_

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <docking/SegmentLineConfig.h>
#include <docking/Line.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/cloud_iterator.h>

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
        startPub();
        startSub();
        startDynamicReconfigureServer();
        initParams();
//        pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/dockLines", 1);
//        sub_  = nh_.subscribe("/cloud", 1, &SegmentLineNode::cloudCallback, this);
      }
      ~SegmentLineNode(){}

      ros::Publisher pub_;
      ros::Subscriber sub_;
      ros::NodeHandle nh_;
      //! Dynamic reconfigure server.
      dynamic_reconfigure::Server<docking::SegmentLineConfig> dr_srv_;

      void startDynamicReconfigureServer(){
        // Set up a dynamic reconfigure server.
        // Do this before parameter server, else some of the parameter server values can be overwritten.
        dynamic_reconfigure::Server<docking::SegmentLineConfig>::CallbackType cb;
        cb = boost::bind(&SegmentLineNode::configCallback, this, _1, _2);
        dr_srv_.setCallback(cb);
      }

      void initParams(){
        // Initialize node parameters from launch file or command line.
        nh_.param("world_frame", world_frame_, world_frame_);
        nh_.param("robot_frame", robot_frame_, robot_frame_);
        nh_.param("cloud_frame", cloud_frame_, cloud_frame_);
        nh_.param("laser_frame", laser_frame_, laser_frame_);

        nh_.param("RS_max_iter", RS_max_iter_, RS_max_iter_);
        nh_.param("RS_dist_thresh", RS_dist_thresh_, RS_dist_thresh_);
        nh_.param("Voxel_leaf_size", Voxel_leaf_size_, Voxel_leaf_size_);

        nh_.param("EC_cluster_tolerance", EC_cluster_tolerance_, EC_cluster_tolerance_);
        nh_.param("EC_min_size", EC_min_size_, EC_min_size_);
        nh_.param("EC_max_size", EC_max_size_, EC_max_size_);



        //Use a private node handle so that multiple instances
        // of the node can be run simultaneously while using different parameters.
//        ros::NodeHandle pnh("~");
//        pnh.param("RS_max_iter", RS_max_iter_, RS_max_iter_);
//        pnh.param("RS_dist_thresh", RS_dist_thresh_, RS_dist_thresh_);
//        pnh.param("Voxel_leaf_size", Voxel_leaf_size_, Voxel_leaf_size_);
      }

      //! Callback function for dynamic reconfigure server.
      void configCallback(docking::SegmentLineConfig &config, uint32_t level __attribute__((unused))){
        RS_max_iter_ = config.RS_max_iter;
        RS_dist_thresh_ = config.RS_dist_thresh;
        Voxel_leaf_size_ = config.Voxel_leaf_size;

      }

      void startPub(){
        pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/dockLines", 1);
      }

      void startSub(){
        sub_  = nh_.subscribe("/cloud", 1, &SegmentLineNode::cloudCallback, this);
      }

      typename pcl::PointCloud<PointT>::Ptr input;
      typename pcl::PointCloud<PointT>::Ptr output;

      void setInputCloud(typename pcl::PointCloud<PointT>::Ptr in)
      {
        input = in;
      }

      void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
      {
        ROS_INFO_STREAM("Callback Called: ");
          // Container for original & filtered data
          /*
          * CONVERT POINTCLOUD ROS->PCL
          */
          pcl::PointCloud<PointT> cloudPCL;
          pcl::fromROSMsg (*msg, cloudPCL);
          typename pcl::PointCloud<PointT>::Ptr cloudPCLPtr (new pcl::PointCloud<PointT> (cloudPCL));
          /////////

          ROS_INFO_STREAM("CALLING RANSAC: ");
          ROS_INFO_STREAM("Distance Threshold: " << RS_dist_thresh_ << " Max Iterations: " << RS_max_iter_);
//          std::vector<typename pcl::PointCloud<PointT>::Ptr> lines;
          typename pcl::PointCloud<PointT>::Ptr lines;
          lines = this->RansacLine(cloudPCLPtr, RS_max_iter_, RS_dist_thresh_);
          ROS_INFO_STREAM("RANSAC COMPLETE");

          /* ========================================
           * CONVERT POINTCLOUD PCL->ROS
           * PUBLISH CLOUD
           * ========================================*/
          sensor_msgs::PointCloud2::Ptr cloudROSPtr (new sensor_msgs::PointCloud2);
//          pcl::toROSMsg(*lines.at(0), *cloudROSPtr);
          pcl::toROSMsg(*lines, *cloudROSPtr);
          cloudROSPtr->header.frame_id = msg->header.frame_id;
      //    cloudROSPtr->header.stamp=ros::Time::now();
          pub_.publish(cloudROSPtr);
      }


      ///////////////// BEGIN RANSAC LINE /////////////////
      /// \brief RansacLine
      /// \param inputCloudPtr
      /// \param maxIterations
      /// \param distanceThreshold
      /// \return lines
      ///
      typename pcl::PointCloud<PointT>::Ptr RansacLine(typename pcl::PointCloud<PointT>::Ptr inputCloudPtr, int maxIterations, float distanceThreshold)
//     std::vector<typename pcl::PointCloud<PointT>::Ptr> RansacLine(typename pcl::PointCloud<PointT>::Ptr inputCloudPtr, int maxIterations, float distanceThreshold)
//     std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> RansacLine(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
    {
       ROS_INFO_STREAM("RANSAC Called: ");
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
      int i = 0;
      while(inCloudPtr->size() != 0){
        std::cout <<  std::endl <<"Detected Line # " << ++i << std::endl;
        if (i > 100) { break; }
        //Segment the largest line component from the input cloud
        seg.setInputCloud(inCloudPtr);
        seg.segment(*inliers, *coefficients);

        segResult = SeparateClouds(inliers, inCloudPtr);

        typename pcl::PointCloud<PointT>::Ptr line (new pcl::PointCloud<PointT>);
        *line = *segResult.first;
        line->width = inliers->indices.size();
        line->height = 1;
        line->is_dense = true;
        lines.push_back(line);
        std::cout << "Coloring Line at index " << lines.size()-1 << " Total lines: " << lines.size() << std::endl;
        ColorCloud(lines.at(lines.size()-1), lines.size()-1);
        inCloudPtr = segResult.second;
        if (inliers->indices.size() <= minInliers) {
          std::cout << std::endl << "Less than " << minInliers << " inliers left" << std::endl;
          break;
        }
      }

      std::cout << "RANSAC: " << lines.size() << " lines found" << std::endl;
      typename std::vector<typename pcl::PointCloud<PointT>::Ptr>::iterator it;

      pcl::PointCloud<PointT> colorLines;
      for (int li = 0; li < lines.size(); li++){
        colorLines += (*lines.at(li)) ;
      }

      typename pcl::PointCloud<PointT>::Ptr colorLinesPtr (new pcl::PointCloud<PointT> (colorLines));

//      auto endTime = std::chrono::steady_clock::now();
//      auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
//      std::cout << "RANSAC took " << elapsedTime.count() << " microseconds" << std::endl;

//      std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> line(coefficients, inliers);
      ROS_INFO_STREAM("RANSAC RETURNING FOUND LINES ");
//      return lines;

      return colorLinesPtr;
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
          extract.setNegative (true);     // Extract the outliers, not inliers
          extract.filter (*outCloud);    //Output cloud
          std::cerr << "Inliers: " << inCloud->width * inCloud->height << " points " << " Outliers: " << outCloud->width * outCloud->height << " points" << std::endl;

          std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(inCloud, outCloud);
          return segResult;
      }
///////////////// END SEPARATE CLOUDS /////////////////


      ///////////////// BEGIN CLUSTER POINTS /////////////////
      /// \brief ClusterPoints
      /// \param cloud
      /// \param clusterTolerance
      /// \param minSize
      /// \param maxSize
      /// \return
      ///
      typename pcl::PointCloud<PointT>::Ptr ClusterPoints(typename pcl::PointCloud<PointT>::Ptr cloud, double clusterTolerance, int minSize, int maxSize)
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
///////////////// END CLUSTER POINTS /////////////////

      ///////////////// BEGIN CENTROID/////////////////
      /// \brief Centroid
      /// \param inCloudPtr
      /// \return
      ///
     Eigen::Vector4f Centroid(typename pcl::PointCloud<PointT>::Ptr inCloudPtr)
      {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*inCloudPtr, centroid);
        std::cout << "The XYZ coordinates of the centroid are: ("
              << centroid[0] << ", "
              << centroid[1] << ", "
              << centroid[2] << ")." << std::endl;

        return centroid;
      }
///////////////// END CENTROID /////////////////

     ///////////////// BEGIN COLOR CLOUD/////////////////
     /// \brief Centroid
     /// \param inCloudPtr
     /// \return
     ///
    void ColorCloud(typename pcl::PointCloud<PointT>::Ptr cPtr, int color)
     {
        color = color % 4;
        for(size_t i = 0; i < cPtr->points.size(); ++i){
          int rgb = cPtr->points[i].rgb;

//          std::cout << "BEF RGB " << cPtr->points[i].rgb << " R " << cPtr->points[i].r << " G " << cPtr->points[i].g << " B " << cPtr->points[i].b << " val " << val <<std::endl;
//          ROS_INFO_STREAM("BEF RGB " << int(cPtr->points[i].rgb) << " R " << unsigned(cPtr->points[i].r )<< " G " << +(cPtr->points[i].g) << " B " << +cPtr->points[i].b);
//          ROS_INFO_STREAM("CUR cit " << " R " << r << " G " << g << " B- " << b);
//          ROS_INFO_STREAM("CUR cit " << " R- " << static_cast<unsigned char>(cit->r) << " G- " << static_cast<unsigned char>(cit->g) << " B- " << (cit->b));
//          std::cout << "CUR cit "<< " R: " << static_cast<unsigned char>(cit->r) << " G: " << cit->g << " B: " << cit->b << std::endl;

          std::uint32_t rgb32;
          switch(color){
            case 0:
              ROS_INFO_STREAM("COLORING RED");
              rgb32 = getRGBUI32(255,0,0);
              break;
            case 1:
              ROS_INFO_STREAM("COLORING GREEN");
              rgb32 = (getRGBUI32(0,255,0));
              break;
            case 2:
              ROS_INFO_STREAM("COLORING BLUE");
              rgb32 = (getRGBUI32(0,0,255));
              break;
            case 3:
              ROS_INFO_STREAM("COLORING PINK");
              rgb32 = (getRGBUI32(255,20,147));
              break;
          }
            cPtr->points[i].rgb = *reinterpret_cast<float*>(&rgb32) ;
        }
     }
///////////////// END COLOR CLOUD /////////////////

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
        voxel.setLeafSize(Voxel_leaf_size_, Voxel_leaf_size_, Voxel_leaf_size_);
//        voxel.setLeafSize(leafSize, leafSize, leafSize);
        voxel.filter(*filteredCloudPtr);

      }
///////////////// END VOXEL GRID /////////////////

    std::uint32_t getRGBUI32(int ri, int gi, int bi){
//      std::uint8_t r(ri), g(gi), b(bi);
//      std::uint32_t rgb32 = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
// ----------------------------------------------------------------------------------------
//      std::uint32_t rgb32 = ( static_cast<std::uint32_t>(r) << 16 |
//                            static_cast<std::uint32_t>(g) << 8 |
//                            static_cast<std::uint32_t>(b)
//                          );
// ----------------------------------------------------------------------------------------
      std::uint32_t rgb32 = ((std::uint32_t)ri << 16 | (std::uint32_t)gi << 8 | (std::uint32_t)bi);
//      int rgb = ((int)ri) << 16 | ((int)gi) << 8 | ((int)bi);
//      std::cout << "CONVERT RGB " << rgb << " R " << unsigned(r) << " G " << unsigned(g) << " B " << unsigned(b) <<std::endl;
      std::cout << "CONVERT RGB " << rgb32 << " Ri " << ri << " Gi " << gi << " Bi " << bi <<std::endl;
      return rgb32;
    }

    void extractRGBUI32(int rgb){
      std::uint8_t r = (rgb >> 16) & 0x0000ff;
      std::uint8_t g = (rgb >> 8)  & 0x0000ff;
      std::uint8_t b = (rgb)     & 0x0000ff;

      std::cout << "EXTRACT RGB " << +rgb << " R " << int(r) << " G " << unsigned(g) << " B " << +b << std::endl;
    }

      //! RANSAC Maximum Iterations
      int RS_max_iter_;
      //! RANSAC Distance Threshold
      double RS_dist_thresh_;

      //! EuclideanCluster Tolerance (m)
      double EC_cluster_tolerance_;
      //! EuclideanCluster Min Cluster Size
      int EC_min_size_;
      //! EuclideanCluster Max Cluster Size
      int EC_max_size_;

      //! Leaf Size for Voxel Grid
      double Voxel_leaf_size_;

      //! Name of world frame
      std::string world_frame_;
      //! Name of robot frame
      std::string robot_frame_;
      //! Name of cloud frame
      std::string cloud_frame_;
      //! Name of cloud topic
      std::string cloud_topic_;
      //! Name of laser frame
      std::string laser_frame_;
      //! Name of laser topic
      std::string laser_topic_;
  };

//  template class SegmentLineNode<pcl::PointXYZ>;
//  template class SegmentLineNode<pcl::PointXYZI>;
  template class SegmentLineNode<pcl::PointXYZRGB>;
//  template class SegmentLineNode<pcl::PointXYZRGBA>;

}// END NAMESPACE DOCKING


//#include <docking/impl/SegmentLineNode.hpp>
//#define PCL_INSTANTIATE_SegmentLineNode(T) template class PCL_EXPORTS pcl::SegmentLineNode<T>;



#endif /*"SEGMENTLINENODE_H_"*/
