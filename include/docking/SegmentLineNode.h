// -*- mode: c++ -*-
#ifndef SEGMENTLINENODE_H_
#define SEGMENTLINENODE_H_

#include <docking/Helpers.h>
#include <docking/PCLHelpers.h>
// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <docking/BoundingBox.h>
#include <docking/Cluster.h>
#include <docking/ClusterArray.h>
#include <docking/Dock.h>
#include <docking/Line.h>
#include <docking/LineArray.h>
#include <docking/MinMaxPoint.h>
#include <docking/SegmentLineConfig.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/Segment.h>
#include <jsk_recognition_msgs/SegmentArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

#include <pcl/cloud_iterator.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_cloud.h>
//#include <pcl/>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace docking {

template <typename PointT> class SegmentLineNode {

public:
  SegmentLineNode(ros::NodeHandle nh) : nh_(nh) {
    startDynamicReconfigureServer();
    initParams();
    startPub();
    startSub(cloud_topic_);
  }
  ~SegmentLineNode() {}
  ros::Publisher clusters_cloud_pub_;
  ros::Publisher clusters_pub_;
  ros::Publisher lines_cloud_pub_;
  ros::Publisher lines_pub_;
  ros::Publisher line_marker_pub_;
  ros::Publisher line_segment_pub_;
  ros::Publisher dock_marker_pub_;
  ros::Publisher bbox_pub_;     // Bounding box publisher
  ros::Publisher jsk_bbox_pub_; // Bounding box publisher
  ros::Publisher debug_pub_;    // debug publisher
  ros::Subscriber sub_;
  ros::NodeHandle nh_;
  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<docking::SegmentLineConfig> dr_srv_;

  //! Input PCL Cloud
  typename pcl::PointCloud<PointT> scanCloudPCL_;
  //! Input PCL Cloud Pointer
  typename pcl::PointCloud<PointT>::Ptr scanCloudPCLPtr_(typename pcl::PointCloud<PointT> scanCloudPCL_);

  void startDynamicReconfigureServer() {
    // Set up a dynamic reconfigure server.
    // Do this before parameter server, else some of the parameter server values
    // can be overwritten.
    dynamic_reconfigure::Server<docking::SegmentLineConfig>::CallbackType cb;
    cb = boost::bind(&SegmentLineNode::configCallback, this, _1, _2);
    dr_srv_.setCallback(cb);
  }

  void initLineMarker(){
    lines_marker_.type = visualization_msgs::Marker::LINE_LIST;
    lines_marker_.header = header_;
    lines_marker_.ns = "docking";
    lines_marker_.id = 0;
  }

  void initParams() {
    // Initialize node parameters from launch file or command line.
    nh_.param("world_frame", world_frame_, world_frame_);
    nh_.param("robot_frame", robot_frame_, robot_frame_);
    nh_.param("cloud_frame", cloud_frame_, cloud_frame_);
    nh_.param("cloud_topic", cloud_topic_, cloud_topic_);
    nh_.param("laser_frame", laser_frame_, laser_frame_);
    nh_.param("laser_topic", laser_topic_, laser_topic_);

    nh_.param("RS_max_iter", RS_max_iter_, RS_max_iter_);
    nh_.param("RS_min_inliers", RS_min_inliers_, RS_min_inliers_);
    nh_.param("RS_dist_thresh", RS_dist_thresh_, RS_dist_thresh_);
    nh_.param("RANSAC_on_clusters", RANSAC_on_clusters_, RANSAC_on_clusters_);

    nh_.param("Voxel_leaf_size", Voxel_leaf_size_, Voxel_leaf_size_);

    nh_.param("EC_cluster_tolerance", EC_cluster_tolerance_,EC_cluster_tolerance_);
    nh_.param("EC_min_size", EC_min_size_, EC_min_size_);
    nh_.param("EC_max_size", EC_max_size_, EC_max_size_);

    nh_.param("CL_centroid_delta", CL_centroid_delta_, CL_centroid_delta_);
    nh_.param("CL_coefficient_delta_", CL_coefficient_delta_, CL_coefficient_delta_);
    nh_.param("CL_segment_delta_", CL_segment_delta_, CL_segment_delta_);
    nh_.param("CL_points_delta_", CL_points_delta_, CL_points_delta_);

    // Use a private node handle so that multiple instances
    // of the node can be run simultaneously while using different parameters.
    //        ros::NodeHandle pnh("~");
    //        pnh.param("RS_max_iter", RS_max_iter_, RS_max_iter_);
    //        pnh.param("RS_dist_thresh", RS_dist_thresh_, RS_dist_thresh_);
    //        pnh.param("Voxel_leaf_size", Voxel_leaf_size_, Voxel_leaf_size_);
  }

  //! Callback function for dynamic reconfigure server.
  void configCallback(docking::SegmentLineConfig &config,
                      uint32_t level __attribute__((unused))) {
    RS_max_iter_ = config.RS_max_iter;
    RS_min_inliers_ = config.RS_min_inliers;
    RS_dist_thresh_ = config.RS_dist_thresh;
    RANSAC_on_clusters_ = config.RANSAC_on_clusters;
    Voxel_leaf_size_ = config.Voxel_leaf_size;
    EC_cluster_tolerance_ = config.EC_cluster_tolerance;
    EC_min_size_ = config.EC_min_size;
    EC_max_size_ = config.EC_max_size;

    CL_centroid_delta_ = config.CL_centroid_delta;
    CL_coefficient_delta_ = config.CL_coefficient_delta;
    CL_segment_delta_ = config.CL_segment_delta;
    CL_points_delta_ = config.CL_points_delta;
    CL_total_delta_ = config.CL_total_delta;

    if (config.cloud_topic != cloud_topic_) {
      cloud_topic_ = config.cloud_topic;
      ROS_INFO_STREAM("New Input Cloud Topic");
      startSub(cloud_topic_);
    }
  }

  void startPub() {
    clusters_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("docking/clustersCloud", 1);
    clusters_pub_ = nh_.advertise<docking::ClusterArray>("docking/clusters", 1);

    lines_cloud_pub_ =  nh_.advertise<sensor_msgs::PointCloud2>("docking/linesCloud", 1);
    lines_pub_ = nh_.advertise<docking::LineArray>("docking/lines", 1);
    line_segment_pub_ = nh_.advertise<jsk_recognition_msgs::SegmentArray>("docking/segments_marker", 1);
    line_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("docking/lines_marker", 1);

    dock_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("docking/dock_marker", 1);
    bbox_pub_ = nh_.advertise<docking::BoundingBox>("docking/bbox", 1);
    jsk_bbox_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBox>("docking/jsk_bbox", 1);

    debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("docking/debugCloud", 1);
  }

  void startSub(std::string cloud_topic) {
    cloud_topic = "/" + cloud_topic;
    ROS_INFO_STREAM("Subscribing to new cloud topic " + cloud_topic_);
    sub_ = nh_.subscribe(cloud_topic, 1, &SegmentLineNode::cloudCallback, this);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    header_ = msg->header;
    segments_.header = lines_.header = lines_marker_.header = header_;
    segments_.segments.clear();
    lines_.lines.clear();
    lines_marker_.points.clear();
    lines_marker_.colors.clear();

    ROS_INFO_STREAM("Callback Called: ");
    // Container for original & filtered data
    /*
     * CONVERT POINTCLOUD ROS->PCL
     */
    pcl::PointCloud<PointT> cloudPCL;
    pcl::fromROSMsg(*msg, cloudPCL);
    pcl::fromROSMsg(*msg, scanCloudPCL_);
    typename pcl::PointCloud<PointT>::Ptr cloudPCLPtr(new pcl::PointCloud<PointT>(cloudPCL));

    /* ========================================
     * CLUSTERING
     * ========================================*/
    docking::ClusterArray clusters = this->ClusterPoints(cloudPCLPtr, EC_cluster_tolerance_, EC_min_size_, EC_max_size_);

    ROS_INFO_STREAM("clusters.combinedCloud.frame_id " << clusters.combinedCloud.header.frame_id);

    /* ========================================
     * RANSAC LINES
     * ========================================*/

    ROS_INFO_STREAM("Distance Threshold: "<< RS_dist_thresh_ << " Max Iterations: " << RS_max_iter_);
    docking::LineArray lines;
    if (RANSAC_on_clusters_) {
      /* ========================================
       * RANSAC LINES FROM CLUSTERS
       * ========================================*/
      lines = this->getRansacLinesOnCluster(clusters);
      ROS_INFO_STREAM("CALLING RANSAC ON CLUSTERED CLOUD: ");

    } else {
      /* ========================================
       * RANSAC on WHOLE CLOUD
       * ========================================*/
      ROS_INFO_STREAM("RAN-CLUS--CALLING RANSAC ON WHOLE CLOUD: ");

      lines = this->getRansacLines(cloudPCLPtr);
    }
    ROS_INFO_STREAM("RAN-CLUS-- COMPLETE");

    /* ========================================
     * PUBLISH CLOUD
     * ========================================*/

    clusters_cloud_pub_.publish(clusters.combinedCloud);
    clusters_pub_.publish(clusters);

    lines_cloud_pub_.publish(lines.combinedCloud);
    //          docking::LineArray::Ptr linesPtr (new docking::LineArray(lines));
    lines_pub_.publish(lines);

    docking::Cluster dockCluster = clusters.clusters.front();
    //          ROS_INFO_STREAM("CALLBACK: Publishing cluster with centroid " <<
    //          pubCluster.centroid);

    if(segments_.segments.size() != 0){
      ROS_INFO_STREAM("CALLBACK: Publishing " << segments_.segments.size() << " SEGMENTS ");
      line_marker_pub_.publish(lines_marker_);
      line_segment_pub_.publish(segments_);
    }
    dock_marker_pub_.publish(dockCluster.bbox.marker);
    bbox_pub_.publish(dockCluster.bbox);
    jsk_bbox_pub_.publish(bboxToJSK(dockCluster.bbox));

    ROS_INFO_STREAM("CALLBACK: CALLBACK COMPLETE");
    std::cout << std::endl;
  }

  ///////////////// BEGIN RANSAC LINE /////////////////
  /// \brief RansacLine
  /// \param inputCloudPtr
  /// \param maxIterations
  /// \param distanceThreshold
  /// \return lines
  ///
  docking::LineArray getRansacLines(typename pcl::PointCloud<PointT>::Ptr inputCloudPtr)
  //     std::vector<typename pcl::PointCloud<PointT>::Ptr> RansacLine(typename
  //     pcl::PointCloud<PointT>::Ptr inputCloudPtr, int maxIterations, float
  //     distanceThreshold) std::pair<pcl::ModelCoefficients::Ptr,
  //     pcl::PointIndices::Ptr> RansacLine(typename
  //     pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float
  //     distanceThreshold)
  {
    typename pcl::PointCloud<PointT> copyInputCloud = *inputCloudPtr;
    typename pcl::PointCloud<PointT>::Ptr copyInputCloudPtr(new pcl::PointCloud<PointT>(copyInputCloud));
    std::cout << std::endl;
    ROS_INFO_STREAM("RANSAC Called: ");
    pcl::ModelCoefficients::Ptr coefficientsPtr(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliersPtr(new pcl::PointIndices());

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(RS_max_iter_);
    seg.setDistanceThreshold(RS_dist_thresh_);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> lineVector;

    docking::LineArray lines;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    typename pcl::PointCloud<PointT>::Ptr inCloudPtr, outCloudPtr;
    inCloudPtr = copyInputCloudPtr;
    pcl::PointCloud<PointT> combinedLinesCloud;
    int i = 0;
    while (true) {

      if (inCloudPtr->size() <= RS_min_inliers_) {
        ROS_INFO_STREAM(std::endl << "Less than " << RS_min_inliers_ << " points left");
        break;
      }

      // Segment the current largest line component from the input cloud
      seg.setInputCloud(inCloudPtr);
      seg.segment(*inliersPtr, *coefficientsPtr);

      if (inliersPtr->indices.size() <= RS_min_inliers_) {
        ROS_INFO_STREAM("Less than " << RS_min_inliers_ << " inliers left");
        break;
      }
      ROS_INFO_STREAM("Detected Line # " << i + 1);
//      ROS_INFO_STREAM("Inlier Points" << inliersPtr->indices << "");
      segResult = SeparateClouds(inliersPtr, inCloudPtr);

      // Create new cloud for detected line
      typename pcl::PointCloud<PointT>::Ptr lineCloudPCLPtr(new pcl::PointCloud<PointT>);
      *lineCloudPCLPtr = *segResult.first;
      lineCloudPCLPtr->width = inliersPtr->indices.size();
      lineCloudPCLPtr->height = 1;
      lineCloudPCLPtr->is_dense = true;

      ROS_INFO_STREAM("Coloring Line at index " << lineVector.size() << " Total lines: " << lineVector.size());
      ColorCloud(lineCloudPCLPtr, i);

      // Add detected line to output cloud and lines array
      lineVector.push_back(lineCloudPCLPtr);
      ROS_INFO_STREAM("Added Line to PCLVector at index "<< lineVector.size() - 1 << " Total lines in Vector: " << lineVector.size());

      combinedLinesCloud += *lineCloudPCLPtr;

      docking::Line line = rosifyLine(lineCloudPCLPtr, inliersPtr, coefficientsPtr);
      line.centroid = getCentroid(lineCloudPCLPtr);
      line.segment = getSegment(lineCloudPCLPtr);
//      compareLines()
      updateSegmentList(line);
      updateLineList(line);
      markLine(line);

      //        ROS_INFO_STREAM("ROSIFYING LINE-- with " <<
      //        line.points.indices.size() << " points and coefficients " <<
      //        line.coefficients);
//      line =
      printLineInfo(line);
      line.header = line.cloud.header = header_;
      ROS_INFO_STREAM("Adding Line MSG at index " << lines.lines.size());
      lines.lines.push_back(line);
      ROS_INFO_STREAM("Total lines in MSG: " << lines.lines.size());

      // Assign cloud of outliers to new input cloud
      inCloudPtr = segResult.second;
      i++;
    }

    ROS_INFO_STREAM("RANSAC: " << lineVector.size() << " lines found");

    pcl::toROSMsg(combinedLinesCloud, lines.combinedCloud);

    ROS_INFO_STREAM("RANSAC RETURNING " << lines.lines.size() << " FOUND LINES ");
    lines.header = lines.combinedCloud.header = header_;
    std::cout << std::endl;
    return lines;
  }
  ////////////////////////////////// END RANSAC LINE
  /////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////////////////

  ///////////////// BEGIN RANSAC LINE ON CLUSTERS /////////////////
  docking::LineArray getRansacLinesOnCluster(docking::ClusterArray clusters) {
    docking::LineArray lines;
    lines.header = lines.combinedCloud.header = header_;
    typename pcl::PointCloud<PointT> linesCombinedPCL;

    //        ROS_INFO_STREAM("lines.combinedCloud.frame_id " <<
    //        lines.combinedCloud.header.frame_id);
    //        sensor_msgs::PointCloud2::Ptr linesCombinedCloudPtr (new
    //        sensor_msgs::PointCloud2(lines.combinedCloud));
    ROS_INFO_STREAM("RAN-CLUS-- " << clusters.clusters.size() << " AVAILABLE CLUSTERS");
    int i = 0;

    for (std::vector<docking::Cluster>::const_iterator cit = clusters.clusters.begin(); cit != clusters.clusters.end(); cit++, i++) {
      ROS_INFO_STREAM("RAN-CLUS--RANSACING THROUGH CLUSTER " << cit->id);
      typename pcl::PointCloud<PointT> cloudPCL;
      typename pcl::PointCloud<PointT>::Ptr cloudPCLPtr(new pcl::PointCloud<PointT>(cloudPCL));
      pcl::fromROSMsg(clusters.clusters.at(i).cloud, *cloudPCLPtr);
      docking::LineArray currentLines = getRansacLines(cloudPCLPtr);
      currentLines.header = currentLines.combinedCloud.header = header_;

      ROS_INFO_STREAM("RAN-CLUS-- DETECTED " << currentLines.lines.size() << " LINES FROM CLUSTER " << cit->id);
      int j = 0;
      for (std::vector<docking::Line>::iterator lit = currentLines.lines.begin(); lit != currentLines.lines.end(); lit++) {
        ROS_INFO_STREAM("RAN-CLUS--ADDING DETECTED LINE " << j + 1);
        currentLines.lines.at(j).clusterID = cit->id;
        lines.lines.push_back(currentLines.lines.at(j));
        j++;
      }

      ROS_INFO_STREAM("RAN-CLUS--COMBINING CLOUDS OF DETECTED LINES FROM CLUSTER " << i + 1);
      typename pcl::PointCloud<PointT> currentLinesCloudPCL;
      pcl::fromROSMsg(currentLines.combinedCloud, currentLinesCloudPCL);
      linesCombinedPCL = linesCombinedPCL + currentLinesCloudPCL;
    }
    ROS_INFO_STREAM("RAN-CLUS--RETURNING ALL DETECTED LINES FROM ALL CLUSTERS");
    pcl::toROSMsg(linesCombinedPCL, lines.combinedCloud);
    lines.header = lines.combinedCloud.header = header_;
//    ROS_INFO_STREAM("lines.combinedCloud.frame_id " << lines.combinedCloud.header.frame_id);
    std::cout << std::endl;
    return lines;
  }
  ///////////////// END RANSAC LINE ON CLUSTERS /////////////////



  ///////////////// BEGIN MATCH DOCK /////////////////

  docking::Dock matchDock(docking::LineArray lines) {
    docking::Dock dock;
    return dock;
  }
  ///////////////// END MATCH DOCK /////////////////

  ///////////////// BEGIN CLUSTER POINTS /////////////////
  /// \brief ClusterPoints
  /// \param cloud
  /// \param clusterTolerance
  /// \param minSize
  /// \param maxSize
  /// \return
  ///
  docking::ClusterArray ClusterPoints(typename pcl::PointCloud<PointT>::Ptr inCloud, double clusterTolerance, int minSize, int maxSize) {
    std::cout << std::endl;
    // ROS_INFO_STREAM();
    ROS_INFO_STREAM("CLUSTERING Called: ");
    //        ROS_INFO_STREAM("");
    ROS_INFO_STREAM("Cluster tolerance: " << clusterTolerance << " Min Points: " << minSize << " Max Points: " << maxSize);

    docking::ClusterArray clusters;

    pcl::PointCloud<PointT> combinedClustersCloud;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(inCloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(inCloud);
    ec.extract(cluster_indices);

    //        std::cout << "CLUSTERING: " << cluster_indices.size() << "
    //        clusters found" << std::endl;
    ROS_INFO_STREAM("CLUSTERING: " << cluster_indices.size()<< " clusters found");

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersPCLVector;
    int i = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++i) {
      typename pcl::PointCloud<PointT>::Ptr cloudClusterPtr(new pcl::PointCloud<PointT>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
        cloudClusterPtr->points.push_back(inCloud->points[*pit]);
      }

      cloudClusterPtr->width = cloudClusterPtr->points.size();
      cloudClusterPtr->height = 1;
      cloudClusterPtr->is_dense = true;

      //          std::cout << "Cluster " << i << " has " <<
      //          cloudClusterPtr->points.size() << " points.\n";
      ROS_INFO_STREAM("Cluster " << i << " has "<< cloudClusterPtr->points.size() << " points");
      // Add current cluster to list of clusters
      //          std::cout << "COLORING CLUSTER\n";
      // ROS_INFO_STREAM();
      ColorCloud(cloudClusterPtr, i);
      //          std::cout << "COMBINING CLUSTER\n";
      // ROS_INFO_STREAM();
      combinedClustersCloud += *cloudClusterPtr;
      //          std::cout << "ADDING CLUSTER TO VECTOR\n";
      // ROS_INFO_STREAM();
      clustersPCLVector.push_back(cloudClusterPtr);
      pcl::PointIndicesPtr currentPointIndicesPtr(new pcl::PointIndices(*it));
      //          std::cout << "ROSIFYING CLUSTER\n";
      // ROS_INFO_STREAM();
      docking::Cluster clusterMsg = rosifyCluster(cloudClusterPtr, currentPointIndicesPtr);
      clusterMsg.id.data = i;
      clusterMsg.header = header_;

      //          clusterMsg.bbox = getBoundingBoxCluster(clusterMsg);
      //          clusterMsg.bbox.pose = getCentroid(cloudClusterPtr);

      clusterMsg.bbox = getBoundingBoxClusterOriented(cloudClusterPtr);
      clusterMsg.bbox.marker = markCluster(clusterMsg);
      clusterMsg.jbbox = bboxToJSK(clusterMsg.bbox);

      clusters.clusters.push_back(clusterMsg);
    }
    //        std::cout << "CONVERTING COMBINED CLUSTER TO ROS MSG\n";
    // ROS_INFO_STREAM();
    pcl::toROSMsg(combinedClustersCloud, clusters.combinedCloud);
    clusters.header = clusters.combinedCloud.header = header_;
    return clusters;
  }
  ///////////////// END CLUSTER POINTS /////////////////

  ///////////////// BEGIN BOUNDING BOX CLUSTER /////////////////
  /// \brief getBoundingBoxCluster
  /// \param cluster
  /// \return bbox
  ///
  docking::BoundingBox getBoundingBoxCluster(docking::Cluster cluster) {
    docking::BoundingBox bbox;

    typename pcl::PointCloud<PointT> cloud;
    pcl::fromROSMsg(cluster.cloud, cloud);
    typename pcl::PointCloud<PointT>::Ptr cloudPtr(
        new pcl::PointCloud<PointT>(cloud));

    // Get Centroid of cluster
    //        bbox.pose.position = getCentroid(cloudPtr);

    Eigen::Vector4f minVec4f;
    Eigen::Vector4f maxVec4f;
    pcl::getMinMax3D(cloud, minVec4f, maxVec4f);

    geometry_msgs::Point point;
    // Get Min Point
    point.x = double(minVec4f[0]);
    point.y = double(minVec4f[1]);
    point.z = double(minVec4f[2]);
    bbox.min = point;
    // Get Max Point
    point.x = double(maxVec4f[0]);
    point.y = double(maxVec4f[1]);
    point.z = double(maxVec4f[2]);
    bbox.max = point;

    bbox.dimensions.x = bbox.max.x - bbox.min.x;
    bbox.dimensions.y = bbox.max.y - bbox.min.y;
    bbox.dimensions.z = bbox.max.z - bbox.min.z;

    bbox.area = (bbox.max.x - bbox.min.x) * (bbox.max.y - bbox.min.y);
    bbox.volume = (bbox.area) * (bbox.max.z - bbox.min.z);

    bbox.header = header_;

    return bbox;
  }
  ///////////////// END BOUNDING BOX CLUSTER /////////////////

  ///////////////// BEGIN BOUNDING BOX CLUSTER ORIENTED /////////////////
  /// \brief getBoundingBoxCluster
  /// \param cluster
  /// \return bbox
  ///
  docking::BoundingBox getBoundingBoxClusterOriented(
      typename pcl::PointCloud<PointT>::Ptr origCloudPtr) {
    //        pcl::PointXYZ minPoint, maxPoint;
    //        pcl::getMinMax3D(*origCloudPtr, minPoint, maxPoint);

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*origCloudPtr, pcaCentroid);
    Eigen::Matrix3f covariance;

    computeCovarianceMatrixNormalized(*origCloudPtr, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
        covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

    /*This line is necessary for proper orientation in some cases.
    The numbers come out the same without it, but the signs are different and
    the box doesn't get correctly oriented in some cases.*/
    eigenVectorsPCA.col(2) =
        eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Note that getting the eigenvectors can also be obtained via the
    // PCL PCA interface with something like:
    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojectionPtr (new
    pcl::PointCloud<pcl::PointXYZ>); pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(origCloudPtr);
    pca.project(*origCloudPtr, *cloudPCAprojectionPtr);
    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() <<
    std::endl; std::cerr << std::endl << "EigenValues: " << pca.getEigenValues()
    << std::endl;
    // In this case, pca.getEigenVectors() gives similar eigenVectors to
    eigenVectorsPCA.
    */

    // Transform the original cloud to the origin where the principal components
    // correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjectedPtr(
        new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*origCloudPtr, *cloudPointsProjectedPtr,
                             projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjectedXYZPtr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloudPointsProjectedPtr, *cloudPointsProjectedXYZPtr);

    printDebugCloud(cloudPointsProjectedXYZPtr);

    //        pcl::PointXYZ minPointProjected, maxPointProjected;
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjectedPtr, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal =
        0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxPosition =
        eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // Convert to ROS BoundingBox Msg
    docking::BoundingBox bboxMsg;

    bboxMsg.pose.position.x = double(bboxPosition[0]);
    bboxMsg.pose.position.y = double(bboxPosition[1]);
    bboxMsg.pose.position.z = double(bboxPosition[2]);

    bboxMsg.pose.orientation.x = double(bboxQuaternion.x());
    bboxMsg.pose.orientation.y = double(bboxQuaternion.y());
    bboxMsg.pose.orientation.z = double(bboxQuaternion.z());
    bboxMsg.pose.orientation.w = double(bboxQuaternion.w());

    geometry_msgs::Point point;
//    ROS_INFO_STREAM("PCL minMaxPoint after getVector3fMap");
//    ROS_INFO_STREAM("PCL minPoint: " << minPoint);
//    ROS_INFO_STREAM("PCL maxPoint: " << maxPoint);
    // Get Min Point
    point.x = double(minPoint.x);
    point.y = double(minPoint.y);
    point.z = double(minPoint.z);
    bboxMsg.min = point;
    // Get Max Point
    point.x = double(maxPoint.x);
    point.y = double(maxPoint.y);
    point.z = double(maxPoint.z);
    bboxMsg.max = point;

    //        @TODO: Transform min and max points back into laser frame
    //        bboxMsg.dimensions.x = bboxMsg.max.x - bboxMsg.min.x ;
    bboxMsg.dimensions.x = 0.1; //
    bboxMsg.dimensions.y = bboxMsg.max.y - bboxMsg.min.y;
    bboxMsg.dimensions.z = bboxMsg.max.z - bboxMsg.min.z;

    bboxMsg.area = bboxMsg.dimensions.x * bboxMsg.dimensions.y;
    bboxMsg.volume = (bboxMsg.area) * bboxMsg.dimensions.z;

//    ROS_INFO_STREAM("bboxMsg.min: " << bboxMsg.min);
//    ROS_INFO_STREAM("bboxMsg.max: " << bboxMsg.max);
//    ROS_INFO_STREAM("bboxMsg.dimensions: " << bboxMsg.dimensions);

    bboxMsg.header = header_;

    return bboxMsg;

    //        visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x -
    //        minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z,
    //        "bbox", mesh_vp_3); addCube (const Eigen::Vector3f &translation,
    //        const Eigen::Quaternionf &rotation,double width, double height,
    //        double depth,const std::string &id = "cube",int viewport = 0);
  }
  ///////////////// END BOUNDING BOX CLUSTER ORIENTED  /////////////////

  ///////////////// BEGIN BBOX DOCKING TO JSK /////////////////
  /// \brief getBoundingBoxCluster
  /// \param cluster
  /// \return bbox
  ///
  jsk_recognition_msgs::BoundingBox bboxToJSK(docking::BoundingBox dbbox) {
    jsk_recognition_msgs::BoundingBox jbbox;
    jbbox.header = header_;
    jbbox.pose = dbbox.pose;
    jbbox.dimensions = dbbox.dimensions;
    //        ROS_INFO_STREAM("dbbox.dimensions: " << dbbox.dimensions);
    //        ROS_INFO_STREAM("jbbox.dimensions: " << jbbox.dimensions);
    jbbox.value = 0.5;
    jbbox.label = 1;

    return jbbox;
  }
  ///////////////// END BBOX DOCKING TO JSK /////////////////




  ///////////////// BEGIN LINE TO SEGMENT /////////////////
  /// \brief getSegment
  /// \param inCloudPtr
  /// \return centroid
  ///
  jsk_recognition_msgs::Segment getSegment(typename pcl::PointCloud<PointT>::Ptr inCloudPtr){
    docking::MinMaxPoint mmp = getMinMaxPointMsg(inCloudPtr);
    jsk_recognition_msgs::Segment segment = minMaxToSegment(mmp);
    return segment;
  }
  ///////////////// END CENTROID /////////////////


  ///////////////// BEGIN LINE TO SEGMENT /////////////////
  /// \brief getCentroid
  /// \param inCloudPtr
  /// \return centroid
  ///
  geometry_msgs::Pose getCentroid(typename pcl::PointCloud<PointT>::Ptr inCloudPtr) {
    Eigen::Vector4f centroidVec4f;
    pcl::compute3DCentroid(*inCloudPtr, centroidVec4f);
    geometry_msgs::Pose centroid;
    centroid.position.x = double(centroidVec4f[0]);
    centroid.position.y = double(centroidVec4f[1]);
    centroid.position.z = double(centroidVec4f[2]);
    centroid.orientation.x = 0.0;
    centroid.orientation.y = 0.0;
    centroid.orientation.z = 0.0;
    centroid.orientation.w = 1.0;
    //        std::cout << "The XYZ coordinates of the centroid are: ("
    //              << centroid.x << ", "
    //              << centroid.y << ", "
    //              << centroid.z << ")." << std::endl;
    return centroid;
  }
  ///////////////// END CENTROID /////////////////

  ///////////////// BEGIN ROSIFY LINE /////////////////
  docking::Line rosifyLine(typename pcl::PointCloud<PointT>::Ptr &cloudPCLPtr,
                           pcl::PointIndices::Ptr &indicesPCLPtr,
                           pcl::ModelCoefficients::Ptr &coefficientsPCLPtr) {
    sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);
    pcl_msgs::PointIndicesPtr indicesMsgPtr(new pcl_msgs::PointIndices);
    pcl_msgs::ModelCoefficientsPtr coefficientsMsgPtr(
        new pcl_msgs::ModelCoefficients);

    pcl::toROSMsg(*cloudPCLPtr, *cloudMsgPtr);
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
  docking::Cluster rosifyCluster(typename pcl::PointCloud<PointT>::Ptr &cloudPCLPtr,
                pcl::PointIndices::Ptr &indicesPCLPtr) {
    sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);
    pcl_msgs::PointIndicesPtr indicesMsgPtr(new pcl_msgs::PointIndices);

    pcl::toROSMsg(*cloudPCLPtr, *cloudMsgPtr);
    //      std::cout << "CONVERTING CURRENT CLUSTER TO ROS MSG\n";
    // ROS_INFO_STREAM();
    docking::Cluster cluster;
    cluster.cloud = *cloudMsgPtr;
    //      std::cout << "CONVERTING INDICES TO ROS MSG\n";
    // ROS_INFO_STREAM();
    cluster.points.indices = indicesPCLPtr->indices;

    return cluster;
  }
  ///////////////// END ROSIFY CLUSTER /////////////////

  ///////////////// BEGIN ROSIFY CLUSTER /////////////////
  docking::Cluster rosifyCluster(typename pcl::PointCloud<PointT>::Ptr &cloudPCLPtr) {
    sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);

    pcl::toROSMsg(*cloudPCLPtr, *cloudMsgPtr);
    //      std::cout << "CONVERTING CURRENT CLUSTER TO ROS MSG\n";
    // ROS_INFO_STREAM();
    docking::Cluster cluster;
    cluster.cloud = *cloudMsgPtr;

    return cluster;
  }
  ///////////////// END ROSIFY CLUSTER /////////////////

  ///////////////// BEGIN PUBLISH MARKER /////////////////
  void publishMarker(typename pcl::PointCloud<PointT>::Ptr &cloudPCLPtr,
                     pcl::PointIndices::Ptr &indicesPCLPtr,
                     pcl::ModelCoefficients::Ptr &coefficientsPCLPtr) {
    sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);
  }
  ///////////////// END PUBLISH MARKER /////////////////

  ///////////////// BEGIN SEPARATE CLOUDS /////////////////
  /// \brief SeparateClouds
  /// \param inliers
  /// \param cloud
  /// \return
  ///
  std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {

    typename pcl::PointCloud<PointT>::Ptr inCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr outCloud(new pcl::PointCloud<PointT>());
    pcl::PointIndices outliers;

    // Obtain the inlier point cloud by copying only inlier indices to the
    // planeCloud for(int index : inliers->indices)
    //     inCloud->points.push_back(cloud->points[index]);
    // ****** ^ does the same as below for the inliers, cuz we already have the
    // inlier indices******
    //Create extract object
    pcl::ExtractIndices<PointT> extract (true);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    // Get plane cloud (inliers)
    extract.setNegative(false); // Extract the inliers, not outliers
    extract.filter(*inCloud);   // Output cloud

    extract.getRemovedIndices(outliers);

    // Get obstacle cloud (outliers)
    extract.setNegative(true); // Extract the outliers, not inliers
    extract.filter(*outCloud); // Output cloud
    //          std::cout << "Inliers: " << inCloud->width * inCloud->height <<
    //          " points " << " Outliers: " << outCloud->width *
    //          outCloud->height << " points" << std::endl;
    ROS_INFO_STREAM("Inliers: " << inCloud->width * inCloud->height << " points Outliers: " << outCloud->width * outCloud->height << " points");
    ROS_INFO_STREAM("Inliers: ");
    printIndices(*inliers);
    ROS_INFO_STREAM("Outliers: ");
    printIndices(outliers);

    std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr>segResult(inCloud, outCloud);
    return segResult;
  }
  ///////////////// END SEPARATE CLOUDS /////////////////

  void printIndices(pcl::PointIndices indices) {
     for(size_t i=0; i < indices.indices.size(); i++)
        std::cout << indices.indices.at(i) << ' ';
     std::cout << std::endl;
  }

  ///////////////// BEGIN COMBINE CLOUDS /////////////////
  /// \brief CombineClouds      std::vector of PointT Clouds
  /// \param lineCloudVector
  /// \param combinedCloud
  ///
  void CombineClouds(
      std::vector<typename pcl::PointCloud<PointT>::Ptr> lineCloudVector,
      typename pcl::PointCloud<PointT>::Ptr combinedCloud) {
    for (int i = 0; i < lineCloudVector.size(); i++) {
      *combinedCloud += (*lineCloudVector.at(i));
    }
  }

  /////
  /// \brief CombineClouds sensor_msgs::PointCloud2
  /// \param augendCloudMsgPtr    Main cloud
  /// \param addendCloudMsgPtr    Cloud to be added to Main cloud
  ///
  sensor_msgs::PointCloud2
  CombineClouds(sensor_msgs::PointCloud2 augendCloudMsg,
                sensor_msgs::PointCloud2 addendCloudMsg) {

    typename pcl::PointCloud<PointT> augendCloudPCL;
    //      ROS_INFO_STREAM("CONVERTING AUGENDCLOUDMSG TO PCL");
    pcl::fromROSMsg(augendCloudMsg, augendCloudPCL);

    typename pcl::PointCloud<PointT>::Ptr addendCloudPCLPtr(
        new pcl::PointCloud<PointT>);
    // ROS_INFO_STREAM("CONVERTING ADDENDCLOUDMSG TO PCL");
    pcl::fromROSMsg(addendCloudMsg, *addendCloudPCLPtr);

    // ROS_INFO_STREAM("ADDING PCL DETECTED LINE CLUSTERS");
    augendCloudPCL += *addendCloudPCLPtr;
    // ROS_INFO_STREAM("CONVERTING COMBINED LINE CLUSTER CLOUD TO ROS MSG");
    pcl::toROSMsg(augendCloudPCL, augendCloudMsg);
    return augendCloudMsg;
  }
  ///////////////// END COMBINE CLOUDS /////////////////

  ///////////////// BEGIN VOXEL GRID /////////////////
  /// \brief VoxelGrid
  /// \param inCloudPtr
  /// \param leafSize
  /// \return
  ///
  void VoxelGrid(typename pcl::PointCloud<PointT>::Ptr inCloudPtr,
                 float leafSize,
                 typename pcl::PointCloud<PointT>::Ptr filteredCloudPtr) {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(inCloudPtr);
    voxel.setLeafSize(Voxel_leaf_size_, Voxel_leaf_size_, Voxel_leaf_size_);
    voxel.filter(*filteredCloudPtr);
  }

  ///////////////// END VOXEL GRID /////////////////




  void printDebugCloud(typename pcl::PointCloud<pcl::PointXYZ>::Ptr debugCloudPtr) {
    sensor_msgs::PointCloud2 debugCloudMsg;
    pcl::toROSMsg(*debugCloudPtr, debugCloudMsg);
    debugCloudMsg.header = header_;
    debug_pub_.publish(debugCloudMsg);
  }

  ///////////////// BEGIN MARK CLUSTER /////////////////
  //    visualization_msgs::Marker
  //    mark_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster,
  //    std::string ns ,int id, float r, float g, float b)
  visualization_msgs::Marker markCluster(docking::Cluster cluster) {
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    marker.header = header_;
    marker.ns = "docking";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = cluster.bbox.pose;
    marker.scale.x = cluster.bbox.dimensions.x;
    marker.scale.y = cluster.bbox.dimensions.y;
    marker.scale.z = cluster.bbox.dimensions.z;

    //      if (marker.scale.x == 0)
    //          marker.scale.x = 0.1;
    //      if (marker.scale.y == 0)
    //        marker.scale.y = 0.1;
    //      if (marker.scale.z == 0)
    //        marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.15f;

    marker.lifetime = ros::Duration();
    //       marker.lifetime = ros::Duration(0.5);
    return marker;
  }

  ///////////////// BEGIN MARK LINE /////////////////
  //    visualization_msgs::Marker
  //    mark_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster,
  //    std::string ns ,int id, float r, float g, float b)
  void markLine(docking::Line line) {

    lines_marker_.type = visualization_msgs::Marker::LINE_LIST;
    lines_marker_.header = header_;
    lines_marker_.ns = "docking";
    lines_marker_.id = 0;

    lines_marker_.action = visualization_msgs::Marker::ADD;

//    lines_marker_.pose = line.centroid;
    lines_marker_.pose.orientation.w = 1.0;
    lines_marker_.points.push_back(line.segment.start_point);
    lines_marker_.points.push_back(line.segment.end_point);
    lines_marker_.scale.x = lines_marker_.scale.y = lines_marker_.scale.z = 0.05;

    lines_marker_.color.r = 0.0f;
    lines_marker_.color.g = 1.0f;
    lines_marker_.color.b = 0.0f;
    lines_marker_.color.a = 0.15f;

//    std_msgs::ColorRGBA color = lines_marker_.color;
    lines_marker_.colors.push_back(lines_marker_.color);
    lines_marker_.colors.push_back(lines_marker_.color);

    lines_marker_.lifetime = ros::Duration();
    //       marker.lifetime = ros::Duration(0.5);
  }

    bool compareLines(docking::Line l1, docking::Line l2){
      float totalDelta;

      float centroidDelta = comparePoses(l1.centroid, l2.centroid);
      float coefficientDelta = compareCoefficients(l1.coefficients, l2.coefficients);
      float segmentDelta = compareSegments(l1.segment, l2.segment);
      float pointsDelta = comparePointIndices(l1.points, l2.points);

       ROS_INFO_STREAM("COMPARING LINES");
       ROS_INFO_STREAM("LINE 1"); printLineInfo(l1);
       ROS_INFO_STREAM("LINE 2"); printLineInfo(l2);
       ROS_INFO_STREAM("DELTA - Centroid: " << centroidDelta << " Points: " << pointsDelta << " Coefficients: " << coefficientDelta << " Segment: " << segmentDelta);

       totalDelta = centroidDelta + coefficientDelta + segmentDelta + pointsDelta;
       totalDelta = totalDelta/4;

      if (totalDelta <= CL_total_delta_){
        return true;
      } else {
        return false;
      }
    }

    void updateLineList(docking::Line line){
      lines_.header = header_;
      ROS_INFO_STREAM("COMPARING LINES");
      bool doesExist = false;
      float lineDelta;
      for (size_t i =0; i < lines_.lines.size(); i++){
        doesExist = compareLines(line, lines_.lines.at(i));
//        ROS_INFO_STREAM("COMPARE LINES - DETECTED LINE " << line);
//        ROS_INFO_STREAM("COMPARE LINES - LINE LIST INDEX " << i << " " << lines_.lines.at(i));
        ROS_INFO_STREAM("COMPARE LINES - DELTA: " << lineDelta);
      }

      if(!doesExist){
        ROS_INFO_STREAM("COMPARE LINES - LINE IS UNIQUE, ADDING TO LIST");
        lines_.lines.push_back(line);
      } else if (lines_.lines.size() == 0){
        ROS_INFO_STREAM("COMPARE LINES - LINE IS UNIQUE, ADDING TO LIST");
        lines_.lines.push_back(line);
      } else {
        ROS_INFO_STREAM("COMPARE LINES - LINE ALREADY EXISTS");
      }

      ROS_INFO_STREAM("COMPARE LINES - TOTAL LINES = " << lines_.lines.size());
    }

      void updateSegmentList(docking::Line line){
//        ROS_INFO_STREAM("COMPARING SEGMENTS");
        bool doesExist = false;
        float segmentDelta;
        for (size_t i =0; i < segments_.segments.size(); i++){
          segmentDelta = compareSegments(line.segment, segments_.segments.at(i));
//          ROS_INFO_STREAM("COMPARE SEGMENTS - SEGMENT OF DETECTED LINE " << line.segment);
//          ROS_INFO_STREAM("COMPARE SEGMENTS - SEGMENT OF SEGMENT LIST INDEX " << i << " " << segments_.segments.at(i));
//          ROS_INFO_STREAM("COMPARE SEGMENTS - DELTA: " << segmentDelta);
          if(segmentDelta < CL_segment_delta_){
            doesExist = true;
          }
        }

        if(!doesExist){
//          ROS_INFO_STREAM("COMPARE SEGMENTS - LINE SEGMENT IS UNIQUE, ADDING TO LIST");
          segments_.segments.push_back(line.segment);
        } else if (segments_.segments.size() == 0){
//          ROS_INFO_STREAM("COMPARE SEGMENTS - LINE SEGMENT IS UNIQUE, ADDING TO LIST");
          segments_.segments.push_back(line.segment);
        } else {
//          ROS_INFO_STREAM("COMPARE SEGMENTS - LINE SEGMENT ALREADY EXISTS");
        }

//        ROS_INFO_STREAM("COMPARE SEGMENTS - TOTAL SEGMENTS = " << segments_.segments.size());
      }



  //! Delta for comparing two line centroids
  float CL_centroid_delta_;
  //! Delta for comparing two line coefficients
  float CL_coefficient_delta_;
  //! Delta for comparing two line segments
  float CL_segment_delta_;
  //! Delta for comparing two line points
  float CL_points_delta_;
  //! Delta for comparing two lines
  float CL_total_delta_;

  //! Global list of line segments for publisher
  jsk_recognition_msgs::SegmentArray segments_;
  //! Global list of line segments for publisher
  docking::LineArray lines_;

  visualization_msgs::Marker lines_marker_;



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

  //! Bool of dock search status
  bool found_Dock_;

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

  std_msgs::Header header_;
};

//  template class SegmentLineNode<pcl::PointXYZ>;
//  template class SegmentLineNode<pcl::PointXYZI>;
template class SegmentLineNode<pcl::PointXYZRGB>;
//  template class SegmentLineNode<pcl::PointXYZRGBA>;

} // namespace docking

//#include <docking/impl/SegmentLineNode.hpp>
//#define PCL_INSTANTIATE_SegmentLineNode(T) template class PCL_EXPORTS
// pcl::SegmentLineNode<T>;

#endif /*"SEGMENTLINENODE_H_"*/
