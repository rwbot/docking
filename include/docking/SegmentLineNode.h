#ifndef SEGMENTLINENODE_H_
#define SEGMENTLINENODE_H_

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <docking/SegmentLineConfig.h>
#include <docking/Line.h>
#include <docking/LineArray.h>
#include <docking/Cluster.h>
#include <docking/ClusterArray.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
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
      }
      ~SegmentLineNode(){}
      ros::Publisher clusters_cloud_pub_;
      ros::Publisher clusters_pub_;
      ros::Publisher lines_cloud_pub_;
      ros::Publisher lines_pub_;
      ros::Publisher line_marker_pub_;
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
        nh_.param("RS_min_inliers", RS_min_inliers_, RS_min_inliers_);
        nh_.param("RS_dist_thresh", RS_dist_thresh_, RS_dist_thresh_);
        nh_.param("RANSAC_on_clusters", RANSAC_on_clusters_, RANSAC_on_clusters_);

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
        RS_min_inliers_ = config.RS_min_inliers;
        RS_dist_thresh_ = config.RS_dist_thresh;
        RANSAC_on_clusters_ = config.RANSAC_on_clusters;
        Voxel_leaf_size_ = config.Voxel_leaf_size;
        EC_cluster_tolerance_ = config.EC_cluster_tolerance;
        EC_min_size_ = config.EC_min_size;
        EC_max_size_ = config.EC_max_size;
      }

      void startPub(){
        clusters_cloud_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/clustersCloud", 1);
        clusters_pub_  = nh_.advertise<docking::ClusterArray>("/clusters", 1);
        lines_cloud_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/linesCloud", 1);
        lines_pub_ = nh_.advertise<docking::LineArray>("/lines", 1);
        line_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("debug/line_marker", 1);
      }

      void startSub(){
        sub_  = nh_.subscribe("/cloud", 1, &SegmentLineNode::cloudCallback, this);
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

          /* ========================================
           * CLUSTERING
           * ========================================*/
          docking::ClusterArray clusters = this->ClusterPoints(cloudPCLPtr, EC_cluster_tolerance_, EC_min_size_, EC_max_size_);
          clusters.header.frame_id = clusters.combinedCloud.header.frame_id = msg->header.frame_id;
          ROS_INFO_STREAM("clusters.combinedCloud.frame_id " << clusters.combinedCloud.header.frame_id);

          /* ========================================
           * RANSAC LINES
           * ========================================*/

          ROS_INFO_STREAM("Distance Threshold: " << RS_dist_thresh_ << " Max Iterations: " << RS_max_iter_);
          docking::LineArray lines;
          if(RANSAC_on_clusters_){
            /* ========================================
             * RANSAC LINES FROM CLUSTERS
             * ========================================*/
            lines = this->getRansacLinesOnCluster(clusters, RS_max_iter_, RS_dist_thresh_);
            ROS_INFO_STREAM("CALLING RANSAC ON CLUSTERED CLOUD: ");

          } else {
            /* ========================================
             * RANSAC on WHOLE CLOUD
             * ========================================*/
            ROS_INFO_STREAM("CALLING RANSAC ON WHOLE CLOUD: ");

            lines = this->getRansacLines(cloudPCLPtr, RS_max_iter_, RS_dist_thresh_);
          }
          ROS_INFO_STREAM("RANSAC COMPLETE");


          /* ========================================
           * PUBLISH CLOUD
           * ========================================*/

          clusters_cloud_pub_.publish(clusters.combinedCloud);
          clusters_pub_.publish(clusters);

          lines.header.frame_id = lines.combinedCloud.header.frame_id = msg->header.frame_id;
          lines_cloud_pub_.publish(lines.combinedCloud);
//          docking::LineArray::Ptr linesPtr (new docking::LineArray(lines));
          lines_pub_.publish(lines);
      }


      ///////////////// BEGIN RANSAC LINE /////////////////
      /// \brief RansacLine
      /// \param inputCloudPtr
      /// \param maxIterations
      /// \param distanceThreshold
      /// \return lines
      ///
      docking::LineArray getRansacLines(typename pcl::PointCloud<PointT>::Ptr inputCloudPtr, int maxIterations, float distanceThreshold)
//     std::vector<typename pcl::PointCloud<PointT>::Ptr> RansacLine(typename pcl::PointCloud<PointT>::Ptr inputCloudPtr, int maxIterations, float distanceThreshold)
//     std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> RansacLine(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
    {
       ROS_INFO_STREAM("RANSAC Called: ");
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

      //Create the segmentation object
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_LINE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(maxIterations);
      seg.setDistanceThreshold(distanceThreshold);

      std::vector<typename pcl::PointCloud<PointT>::Ptr > lineVector;

      docking::LineArray lines;
      std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
      typename pcl::PointCloud<PointT>::Ptr inCloudPtr, outCloudPtr;
      inCloudPtr = inputCloudPtr;
      pcl::PointCloud<PointT> combinedLinesCloud;
      int i = 0;
      while(true){

        if (inCloudPtr->size() <= RS_min_inliers_) {
          std::cout << std::endl << "Less than " << RS_min_inliers_ << " points left" << std::endl;
          break;
        }
        ROS_INFO_STREAM("Detected Line # " << i+1);
        std::cout <<"Detected Line # " << i+1 << std::endl;
        //Segment the current largest line component from the input cloud
        seg.setInputCloud(inCloudPtr);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() <= RS_min_inliers_) {
          ROS_INFO_STREAM("Less than " << RS_min_inliers_ << " inliers left");
          std::cout << std::endl << "Less than " << RS_min_inliers_ << " inliers left" << std::endl;
          break;
        }

        segResult = SeparateClouds(inliers, inCloudPtr);

        typename pcl::PointCloud<PointT>::Ptr lineCloudPCLPtr (new pcl::PointCloud<PointT>);
        *lineCloudPCLPtr = *segResult.first;
        lineCloudPCLPtr->width = inliers->indices.size();
        lineCloudPCLPtr->height = 1;
        lineCloudPCLPtr->is_dense = true;

//        std::cout << "Coloring Line at index " << lineVector.size()-1 << " Total lines: " << lineVector.size() << std::endl;
        ColorCloud(lineCloudPCLPtr, i);

        // Add detected line to output cloud and lines array
        lineVector.push_back(lineCloudPCLPtr);

        combinedLinesCloud += *lineCloudPCLPtr;

        docking::Line line = rosifyLine(lineCloudPCLPtr, inliers, coefficients);
        lines.lines.push_back(line);

        // Assign cloud of outliers to new input cloud
        inCloudPtr = segResult.second;
        i++;
      }

      ROS_INFO_STREAM("RANSAC: " << lineVector.size() << " lines found");
      std::cout << "RANSAC: " << lineVector.size() << " lines found" << std::endl;

      pcl::toROSMsg(combinedLinesCloud,lines.combinedCloud);

      ROS_INFO_STREAM("RANSAC RETURNING FOUND LINES ");

      return lines;
      }
///////////////// END RANSAC LINE /////////////////


///////////////// BEGIN RANSAC LINE ON CLUSTERS /////////////////
      docking::LineArray getRansacLinesOnCluster(docking::ClusterArray clusters, int maxIterations, float distanceThreshold){
        docking::LineArray lines;
        typename pcl::PointCloud<PointT> linesCombinedPCL;
        lines.header.frame_id = lines.combinedCloud.header.frame_id = clusters.header.frame_id;
        ROS_INFO_STREAM("lines.combinedCloud.frame_id " << lines.combinedCloud.header.frame_id);
//        sensor_msgs::PointCloud2::Ptr linesCombinedCloudPtr (new sensor_msgs::PointCloud2(lines.combinedCloud));

        for (int i=0; i<clusters.clusters.size(); i++)
        {
          std::cout << "RANSACING THROUGH CLUSTER " << i << "\n";

          typename pcl::PointCloud<PointT> cloudPCL;
          typename pcl::PointCloud<PointT>::Ptr cloudPCLPtr (new pcl::PointCloud<PointT>(cloudPCL));
          pcl::fromROSMsg(clusters.clusters.at(i).cloud, *cloudPCLPtr);
          docking::LineArray currentLines = getRansacLines(cloudPCLPtr, RS_max_iter_, RS_dist_thresh_);
          currentLines.header.frame_id = currentLines.combinedCloud.header.frame_id = lines.header.frame_id;

          std::cout << "ADDING DETECTED LINES FROM CLUSTER " << i << "\n";
          for(int j=0; j < currentLines.lines.size();j++){
            lines.lines.push_back(currentLines.lines.front());
          }

          std::cout << "COMBINING CLOUDS OF DETECTED LINES FROM CLUSTER " << i << "\n";
         typename pcl::PointCloud<PointT> currentLinesCloudPCL;
          pcl::fromROSMsg(currentLines.combinedCloud, currentLinesCloudPCL);
          linesCombinedPCL = linesCombinedPCL + currentLinesCloudPCL;
//          lines.combinedCloud = CombineClouds(lines.combinedCloud, currentLines.combinedCloud);

        }
        std::cout << "RETURNING ALL DETECTED LINES FROM ALL CLUSTERS\n";
        pcl::toROSMsg(linesCombinedPCL,lines.combinedCloud);
        return lines;
      }
///////////////// END RANSAC LINE ON CLUSTERS /////////////////

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
      docking::ClusterArray ClusterPoints(typename pcl::PointCloud<PointT>::Ptr inCloud, double clusterTolerance, int minSize, int maxSize)
      {
        std::cout << std::endl;
        ROS_INFO_STREAM("CLUSTERING Called: ");
        ROS_INFO_STREAM("");
        ROS_INFO_STREAM("Cluster tolerance: " << clusterTolerance << " Min Points: " << minSize << " Max Points: " << maxSize );

        docking::ClusterArray clusters;
        pcl::PointCloud<PointT> combinedClustersCloud;
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (inCloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (clusterTolerance);
        ec.setMinClusterSize (minSize);
        ec.setMaxClusterSize (maxSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud (inCloud);
        ec.extract(cluster_indices);

        std::cout << "CLUSTERING: " << cluster_indices.size() << " clusters found" << std::endl;

        std::vector<typename pcl::PointCloud<PointT>::Ptr > clustersPCLVector;
        int i =0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, ++i)
        {

          typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            cloud_cluster->points.push_back(inCloud->points[*pit]);
          }

          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;
          std::cout << "Cluster " << i << " has " << cloud_cluster->points.size() << " points.\n";
          // Add current cluster to list of clusters
          std::cout << "COLORING CLUSTER\n";
          ColorCloud(cloud_cluster, i);
          std::cout << "COMBINING CLUSTER\n";
          combinedClustersCloud += *cloud_cluster;
          std::cout << "ADDING CLUSTER TO VECTOR\n";
          clustersPCLVector.push_back(cloud_cluster);
          pcl::PointIndicesPtr currentPointIndicesPtr (new pcl::PointIndices(*it));
          std::cout << "ROSIFYING CLUSTER\n";
          docking::Cluster clusterMsg = rosifyCluster(cloud_cluster, currentPointIndicesPtr);

          clusters.clusters.push_back(clusterMsg);
        }
        std::cout << "CONVERTING COMBINED CLUSTER TO ROS MSG\n";
        pcl::toROSMsg(combinedClustersCloud,clusters.combinedCloud);
//        clusters.header
        return clusters;
      }
///////////////// END CLUSTER POINTS /////////////////

      ///////////////// BEGIN CENTROID POINT /////////////////
      /// \brief CentroidPoint
      /// \param inCloudPtr
      /// \return centroid
      ///
     geometry_msgs::Point CentroidPoint(typename pcl::PointCloud<PointT>::Ptr inCloudPtr)
      {
        Eigen::Vector4f centroidVec4f;
        pcl::compute3DCentroid(*inCloudPtr, centroidVec4f);
        geometry_msgs::Point centroid;
        centroid.x = centroidVec4f[0];
        centroid.y = centroidVec4f[1];
        centroid.z = centroidVec4f[2];
        std::cout << "The XYZ coordinates of the centroid are: ("
              << centroid.x << ", "
              << centroid.y << ", "
              << centroid.z << ")." << std::endl;
        return centroid;
      }
///////////////// END CENTROID /////////////////



///////////////// BEGIN ROSIFY LINE /////////////////
    docking::Line rosifyLine(typename pcl::PointCloud<PointT>::Ptr &cloudPCLPtr, pcl::PointIndices::Ptr &indicesPCLPtr, pcl::ModelCoefficients::Ptr &coefficientsPCLPtr){
      sensor_msgs::PointCloud2Ptr cloudMsgPtr (new sensor_msgs::PointCloud2);
      pcl_msgs::PointIndicesPtr indicesMsgPtr (new pcl_msgs::PointIndices);
      pcl_msgs::ModelCoefficientsPtr coefficientsMsgPtr (new pcl_msgs::ModelCoefficients);

      pcl::toROSMsg(*cloudPCLPtr,*cloudMsgPtr);
//      pcl_conversions::moveFromPCL(*indicesPCLPtr,*indicesMsgPtr);
//      pcl_conversions::moveFromPCL(*coefficientsPCLPtr,*coefficientsMsgPtr);

      docking::Line line;
      line.cloud = *cloudMsgPtr;
//      line.points = *indicesMsgPtr;
      line.points.indices = indicesPCLPtr->indices;
      line.coefficients.values = coefficientsPCLPtr->values;

      return line;
    }
///////////////// END ROSIFY LINE /////////////////

///////////////// BEGIN ROSIFY CLUSTER /////////////////
    docking::Cluster rosifyCluster(typename pcl::PointCloud<PointT>::Ptr &cloudPCLPtr, pcl::PointIndices::Ptr &indicesPCLPtr){
      sensor_msgs::PointCloud2Ptr cloudMsgPtr (new sensor_msgs::PointCloud2);
      pcl_msgs::PointIndicesPtr indicesMsgPtr (new pcl_msgs::PointIndices);

      pcl::toROSMsg(*cloudPCLPtr,*cloudMsgPtr);
      std::cout << "CONVERTING CURRENT CLUSTER TO ROS MSG\n";
      docking::Cluster cluster;
      cluster.cloud = *cloudMsgPtr;
      std::cout << "CONVERTING INDICES TO ROS MSG\n";
      cluster.points.indices = indicesPCLPtr->indices;

      return cluster;
    }
///////////////// END ROSIFY CLUSTER /////////////////

///////////////// BEGIN PUBLISH MARKER /////////////////
    void publishMarker(typename pcl::PointCloud<PointT>::Ptr &cloudPCLPtr, pcl::PointIndices::Ptr &indicesPCLPtr, pcl::ModelCoefficients::Ptr &coefficientsPCLPtr){
      sensor_msgs::PointCloud2Ptr cloudMsgPtr (new sensor_msgs::PointCloud2);

    }
///////////////// END PUBLISH MARKER /////////////////


    ///////////////// BEGIN COMBINE CLOUDS /////////////////
    /// \brief CombineClouds      std::vector of PointT Clouds
    /// \param lineCloudVector
    /// \param combinedCloud
    ///
    void CombineClouds(std::vector<typename pcl::PointCloud<PointT>::Ptr> lineCloudVector, typename pcl::PointCloud<PointT>::Ptr combinedCloud){
      for (int i = 0; i < lineCloudVector.size(); i++){
        *combinedCloud += (*lineCloudVector.at(i)) ;
      }
    }

    /////
    /// \brief CombineClouds sensor_msgs::PointCloud2
    /// \param augendCloudMsgPtr    Main cloud
    /// \param addendCloudMsgPtr    Cloud to be added to Main cloud
    ///
    sensor_msgs::PointCloud2 CombineClouds(sensor_msgs::PointCloud2 augendCloudMsg, sensor_msgs::PointCloud2 addendCloudMsg){

      typename pcl::PointCloud<PointT> augendCloudPCL;
      std::cout << "CONVERTING AUGENDCLOUDMSG TO PCL\n";
      pcl::fromROSMsg(augendCloudMsg, augendCloudPCL);


      typename pcl::PointCloud<PointT>::Ptr addendCloudPCLPtr (new pcl::PointCloud<PointT>);
      std::cout << "CONVERTING ADDENDCLOUDMSG TO PCL\n";
      pcl::fromROSMsg(addendCloudMsg, *addendCloudPCLPtr);

      std::cout << "ADDING PCL DETECTED LINE CLUSTERS\n";
      augendCloudPCL += *addendCloudPCLPtr;
      std::cout << "CONVERTING COMBINED LINE CLUSTER CLOUD TO ROS MSG\n";
      pcl::toROSMsg(augendCloudPCL,augendCloudMsg);
      return augendCloudMsg;
    }
///////////////// END COMBINE CLOUDS /////////////////


    ///////////////// BEGIN VOXEL GRID /////////////////
      /// \brief VoxelGrid
      /// \param inCloudPtr
      /// \param leafSize
      /// \return
      ///
    void VoxelGrid(typename pcl::PointCloud<PointT>::Ptr inCloudPtr, float leafSize, typename pcl::PointCloud<PointT>::Ptr filteredCloudPtr)
     {
       pcl::VoxelGrid<PointT> voxel;
       voxel.setInputCloud(inCloudPtr);
       voxel.setLeafSize(Voxel_leaf_size_, Voxel_leaf_size_, Voxel_leaf_size_);
       voxel.filter(*filteredCloudPtr);
     }

///////////////// END VOXEL GRID /////////////////

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
//              ROS_INFO_STREAM("COLORING RED");
             rgb32 = getRGBUI32(255,0,0);
             break;
           case 1:
//              ROS_INFO_STREAM("COLORING GREEN");
             rgb32 = (getRGBUI32(0,255,0));
             break;
           case 2:
//              ROS_INFO_STREAM("COLORING BLUE");
             rgb32 = (getRGBUI32(0,0,255));
             break;
           case 3:
//              ROS_INFO_STREAM("COLORING PINK");
             rgb32 = (getRGBUI32(255,20,147));
             break;
         }
           cPtr->points[i].rgb = *reinterpret_cast<float*>(&rgb32) ;
       }
    }
///////////////// END COLOR CLOUD /////////////////


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
//      std::cout << "CONVERT RGB " << rgb32 << " Ri " << ri << " Gi " << gi << " Bi " << bi <<std::endl;
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
      //! RANSAC Minimum Inliers
      int RS_min_inliers_;
      //! RANSAC Distance Threshold
      double RS_dist_thresh_;
      //! Perform RANSAC after Clustering Points
      bool RANSAC_on_clusters_;

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
