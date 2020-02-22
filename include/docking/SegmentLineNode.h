// -*- mode: c++ -*-
#ifndef SEGMENTLINENODE_H_
#define SEGMENTLINENODE_H_

#include <docking/Headers.h>
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
#include <docking/ICP.h>

#include <docking/Clustering.h>

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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/cloud_iterator.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/pcl_base.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>

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

//namespace docking {

class SegmentLineNode {
///////////////// BEGIN VARIABLES /////////////////
public:
  SegmentLineNode(ros::NodeHandle nh) : nh_(nh) {
    startDynamicReconfigureServer();
    initParams();
    startPub();
    startSub(cloud_topic_);
    initGlobals();
    initDockParams();
  }
  ~SegmentLineNode() {}

  ros::Publisher clusters_cloud_pub_;
  ros::Publisher clusters_pub_;
  ros::Publisher lines_cloud_pub_;
  ros::Publisher lines_pub_;
  ros::Publisher line_marker_pub_;
  ros::Publisher line_segment_pub_;
  ros::Publisher dock_marker_pub_;
  ros::Publisher dock_pose_pub_;
  ros::Publisher bbox_pub_;     // Bounding box publisher
  ros::Publisher jsk_bbox_pub_; // Bounding box publisher
  ros::Publisher debug_pub_;    // debug publisher
  ros::Publisher icp_in_pub_;   // ICP Input Cloud publisher
  ros::Publisher icp_target_pub_; // ICP Target Cloud publisher
  ros::Publisher icp_out_pub_;   // ICP Output Cloud publisher
  ros::Subscriber sub_;
  ros::NodeHandle nh_;
  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<docking::SegmentLineConfig> dr_srv_;

  //! Input PCL Cloud
  pcl::PointCloud<pcl::PointXYZRGB> scanCloudPCL_;
  //! Input PCL Cloud Pointer
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scanCloudPCLPtr_(pcl::PointCloud<pcl::PointXYZRGB> scanCloudPCL_);
  //! Target Dock PCL Cloud Pointer
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dockTargetPCLPtr_;


  //! Global list of line segments for publisher
  jsk_recognition_msgs::SegmentArray segments_;
  //! Global list of line markers for publisher
  visualization_msgs::Marker lines_marker_;
  //! Global list of line msgs for publisher
  docking::LineArray lines_;
  //! Global list of cluster msgs for publisher
  docking::ClusterArray clusters_;
  //! Global list of cluster msgs for publisher
  docking::ClusterArray::Ptr clustersPtr_;

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
  //! Dock Wing Length
  double dock_wing_length;

  //! Dock target template filepath
  std::string dockFilePath_;
  //! ICP Fitness Score Threshold
  double icpScore_;

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

  ///////////////// END VARIABLES /////////////////



  // Declaration and Definition
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
    std::cout << "initParams: INITIALIZING PARAMS FROM PARAM SERVER: " << std::endl;
    // Initialize node parameters from launch file or command line.
    nh_.param("world_frame", world_frame_, world_frame_);
    nh_.param("robot_frame", robot_frame_, robot_frame_);
    nh_.param("cloud_frame", cloud_frame_, cloud_frame_);
    nh_.param("cloud_topic", cloud_topic_, cloud_topic_);
    nh_.param("laser_frame", laser_frame_, laser_frame_);
    nh_.param("laser_topic", laser_topic_, laser_topic_);

    std::cout << "initParams: CURRENT LASER TOPIC: " << laser_topic_ << std::endl;

    std::cout << "initParams: CURRENT CLOUD TOPIC: " << cloud_topic_ << std::endl;

    nh_.param("RS_max_iter", RS_max_iter_, RS_max_iter_);
    nh_.param("RS_min_inliers", RS_min_inliers_, RS_min_inliers_);
    nh_.param("RS_dist_thresh", RS_dist_thresh_, RS_dist_thresh_);
    nh_.param("RANSAC_on_clusters", RANSAC_on_clusters_, RANSAC_on_clusters_);

    nh_.param("Voxel_leaf_size", Voxel_leaf_size_, Voxel_leaf_size_);

    nh_.param("EC_cluster_tolerance", EC_cluster_tolerance_,EC_cluster_tolerance_);
    nh_.param("EC_min_size", EC_min_size_, EC_min_size_);
    nh_.param("EC_max_size", EC_max_size_, EC_max_size_);

    nh_.param("CL_centroid_delta", CL_centroid_delta_, CL_centroid_delta_);
    nh_.param("CL_coefficient_delta", CL_coefficient_delta_, CL_coefficient_delta_);
    nh_.param("CL_segment_delta", CL_segment_delta_, CL_segment_delta_);
    nh_.param("CL_points_delta", CL_points_delta_, CL_points_delta_);

    nh_.param("dock_filepath", dockFilePath_, dockFilePath_);
    nh_.param("icp_score", icpScore_, icpScore_);

    std::cout << "initParams: CURRENT TARGET CLOUD FILE PATH: " << dockFilePath_ << std::endl;

    // Use a private node handle so that multiple instances
    // of the node can be run simultaneously while using different parameters.
    //        ros::NodeHandle pnh("~");
    //        pnh.param("RS_max_iter", RS_max_iter_, RS_max_iter_);
    //        pnh.param("RS_dist_thresh", RS_dist_thresh_, RS_dist_thresh_);
    //        pnh.param("Voxel_leaf_size", Voxel_leaf_size_, Voxel_leaf_size_);
  }

  void initDockParams(){
    //    std::string dockFilePath("/home/rwbot/dock_ws/src/docking/pcd/dock_cloud_perfect.ply");
    //    std::string dockFilePath("UNSPECIFIED");
    ROS_INFO_STREAM("initDockParams: INITIALIZING TARGET CLOUD FILE PATH");

    bool dockFilepathExists = nh_.hasParam("dock_filepath");

    if(dockFilepathExists){
      ROS_INFO_STREAM("initDockParams: PARAM SERVER TARGET CLOUD FILE PATH: " << dockFilePath_);

    //    ROS_INFO_STREAM("initDockParams: ASSIGNING POINT CLOUD POINTER");
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr dockTargetPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

      ROS_INFO_STREAM("initDockParams: LOADING DOCK TARGET CLOUD FILE" << dockFilePath_);
      if(readPointCloudFile(dockFilePath_,dockTargetPCLPtr) == false){
        ROS_ERROR_STREAM("initDockParams: FAILED TO LOAD TARGET DOCK FILE");
      } else {
        ROS_INFO_STREAM("initDockParams: SUCCESSFULLY LOADED TARGET DOCK FILE");
        dockTargetPCLPtr_ = dockTargetPCLPtr;
      }

    } else {

      ROS_ERROR_STREAM("initDockParams: NO PARAMETER dock_filepath EXISTS FOR TARGET CLOUD FILE PATH");
    }
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

    icpScore_ = config.icp_score;

    if (config.cloud_topic != cloud_topic_) {
      cloud_topic_ = config.cloud_topic;
      ROS_INFO_STREAM("configCallback: New Input Cloud Topic");
      startSub(cloud_topic_);
    }

//    if (config.laser_topic != laser_topic_) {
//      laser_topic_ = config.laser_topic;
//      ROS_INFO_STREAM("configCallback: NEW Input Laser Topic" << laser_topic_);
//    }


    if(dockFilePath_==""){
      std::cout << "configCallback: dockFilePath FILEPATH NOT SPECIFIED " << dockFilePath_ << std::endl;
    }
    else if (config.dock_filepath != dockFilePath_) {
      dockFilePath_ = config.dock_filepath;
      ROS_INFO_STREAM("configCallback: New Dock Target File Specified");
      std::cout << "configCallback: CONFIG FILEPATH: " << config.dock_filepath << std::endl;
      std::cout << "configCallback: DOCK TARGET FILEPATH: " << dockFilePath_ << std::endl;
      ROS_INFO_STREAM("configCallback: LOADING DOCK TARGET CLOUD FILE");
      if(readPointCloudFile(dockFilePath_,dockTargetPCLPtr_) == false){
        ROS_ERROR_STREAM("configCallback: FAILED TO LOAD TARGET DOCK FILE");
      } else {
        ROS_WARN_STREAM("configCallback: SUCCESSFULLY LOADED TARGET DOCK FILE");
      }

    }

  }

  void initGlobals(){
//    clusters_ new (docking::ClusterArray());
    docking::ClusterArray temp1 = docking::ClusterArray();
//    docking::ClusterArray::Ptr tempPtr (docking::ClusterArray clusters_);
    docking::ClusterArray::Ptr tempPtr (new docking::ClusterArray ());

//    docking::ClusterArray::Ptr clustersPtr_ ( docking::ClusterArray clusters_);
//    *clustersPtr_ = *tempPtr;
    clustersPtr_ = tempPtr;
    *clustersPtr_ = temp1;
  }

  void startPub() {
    clusters_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("docking/clustersCloud", 1);
    clusters_pub_ = nh_.advertise<docking::ClusterArray>("docking/clusters", 1);

    lines_cloud_pub_ =  nh_.advertise<sensor_msgs::PointCloud2>("docking/linesCloud", 1);
    lines_pub_ = nh_.advertise<docking::LineArray>("docking/lines", 1);
    line_segment_pub_ = nh_.advertise<jsk_recognition_msgs::SegmentArray>("docking/segments_marker", 1);
    line_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("docking/lines_marker", 1);

    dock_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("docking/dock_marker", 1);
    dock_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("docking/dock_pose", 1);
    bbox_pub_ = nh_.advertise<docking::BoundingBox>("docking/bbox", 1);
    jsk_bbox_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBox>("docking/jsk_bbox", 1);

    debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("docking/debugCloud", 1);

    icp_in_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("docking/icp_in_pub", 1);
    icp_target_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("docking/icp_target_pub", 1);
    icp_out_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("docking/icp_out_pub", 1);
  }

  void startSub(std::string cloud_topic) {
    cloud_topic = "/" + cloud_topic;
    ROS_INFO_STREAM("Subscribing to new cloud topic " + cloud_topic_);
    sub_ = nh_.subscribe(cloud_topic, 1, &SegmentLineNode::cloudCallback, this);
  }

  void clearGlobals(){
    segments_.segments.clear();
    lines_.lines.clear();
    lines_marker_.points.clear();
    lines_marker_.colors.clear();
    clustersPtr_->clusters.clear();
  }



  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {

    clearGlobals();

    header_ = msg->header;
    clustersPtr_->header = clusters_.header = segments_.header = lines_.header = lines_marker_.header = header_;

  static tf2_ros::TransformBroadcaster tfbr;


//    clustersPtr_. = &clusters_;
//    clustersPtr_.reset(clusters_);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dockTargetPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

    ROS_INFO_STREAM("CLOUD CALLBACK: CALLBACK CALLED ");

//    printDebugCloud(dockTargetPCLPtr_);

    // Container for original & filtered data
    /*
     * CONVERT POINTCLOUD ROS->PCL
     */
    pcl::PointCloud<pcl::PointXYZRGB> cloudPCL;
    pcl::fromROSMsg(*msg, cloudPCL);
    pcl::fromROSMsg(*msg, scanCloudPCL_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>(cloudPCL));

    /* ========================================
     * CLUSTERING
     * ========================================*/
//    clusters_ = this->ClusterPoints(cloudPCLPtr, EC_cluster_tolerance_, EC_min_size_, EC_max_size_);

//    ROS_INFO_STREAM("CLOUD CALLBACK: clustersPtr_.header " << clustersPtr_->header);

    Clustering clustering;
    clustering.setHeader(header_);
    clustering.ClusterPoints(cloudPCLPtr, clustersPtr_, EC_cluster_tolerance_, EC_min_size_, EC_max_size_);

//    ROS_INFO_STREAM("CLOUD CALLBACK: CLUSTERING RETURNED " << clustersPtr_->clusters.size() << " CLUSTERS");

//    ROS_INFO_STREAM("CLOUD CALLBACK: clusters.combinedCloud.frame_id " << clustersPtr_->combinedCloud.header.frame_id);

    /* ========================================
     * RANSAC LINES
     * ========================================*/

//    ROS_INFO_STREAM("CLOUD CALLBACK: Distance Threshold: "<< RS_dist_thresh_ << " Max Iterations: " << RS_max_iter_);
    docking::LineArray lines;
    if (RANSAC_on_clusters_) {
      /* ========================================
       * RANSAC LINES FROM CLUSTERS
       * ========================================*/
      lines = this->getRansacLinesOnCluster(*clustersPtr_);
//      ROS_INFO_STREAM("CLOUD CALLBACK: CALLING RANSAC ON CLUSTERED CLOUD ");

    } else {
      /* ========================================
       * RANSAC on WHOLE CLOUD
       * ========================================*/
//      ROS_INFO_STREAM("CLOUD CALLBACK: RAN-CLUS--CALLING RANSAC ON WHOLE CLOUD: ");

      lines = this->getRansacLines(cloudPCLPtr);
    }
//    ROS_INFO_STREAM("CLOUD CALLBACK: RAN-CLUS-- COMPLETE");

    /* ========================================
     * PUBLISH CLOUD
     * ========================================*/

    if(clustersPtr_->clusters.size()>0){
      clusters_cloud_pub_.publish(clustersPtr_->combinedCloud);
      clusters_pub_.publish(clustersPtr_);
      lines_cloud_pub_.publish(lines.combinedCloud);
      //          docking::LineArray::Ptr linesPtr (new docking::LineArray(lines));
      lines_pub_.publish(lines);

      // Assign Dock Cluster
      docking::Cluster dockCluster = clustersPtr_->clusters.front();

      dock_marker_pub_.publish(dockCluster.bbox.marker);
      bbox_pub_.publish(dockCluster.bbox);
      jsk_bbox_pub_.publish(clustering.bboxToJSK(dockCluster.bbox));
    }

//    if(clustersPtr_->clusters.size()>0)


    //          ROS_INFO_STREAM("CALLBACK: Publishing cluster with centroid " <<
    //          pubCluster.centroid);

    if(segments_.segments.size() != 0){
//      ROS_INFO_STREAM("CLOUD CALLBACK: Publishing " << segments_.segments.size() << " SEGMENTS ");
      line_marker_pub_.publish(lines_marker_);
      line_segment_pub_.publish(segments_);
    }



    /* ========================================
     * ITERATIVE CLOSEST POINT
     * ========================================*/

    ROS_INFO_STREAM("CLOUD CALLBACK: BEGINNING ITERATIVE CLOSEST POINT");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ICPInputCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ICPOutCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    docking::Cluster::Ptr dockClusterPtr(new docking::Cluster());
//    ICP(ICPInputCloudPtr, dockTargetPCLPtr_, ICPOutCloudPtr);
    bool icpSuccess = clusterArrayICP(clustersPtr_, dockTargetPCLPtr_,dockClusterPtr);


    ICPInputCloudPtr->header.frame_id = dockTargetPCLPtr_->header.frame_id = ICPOutCloudPtr->header.frame_id = header_.frame_id;
    if(icpSuccess){
      icp_in_pub_.publish(dockClusterPtr->cloud);
      icp_out_pub_.publish(dockClusterPtr->icpCombinedCloud);
      dock_pose_pub_.publish(dockClusterPtr->icp.poseStamped);

      dockClusterPtr->icp.transformStamped.header.stamp = ros::Time::now();
      dockClusterPtr->icp.transformStamped.header.frame_id = laser_frame_;
      dockClusterPtr->icp.transformStamped.child_frame_id = "dock";
      tfbr.sendTransform(dockClusterPtr->icp.transformStamped);
    }
    icp_target_pub_.publish(dockTargetPCLPtr_);


    ROS_INFO_STREAM("CLOUD CALLBACK: CALLBACK COMPLETE");
    std::cout << std::endl;
  }

  ///////////////// BEGIN RANSAC LINE /////////////////
  /// \brief RansacLine
  /// \param inputCloudPtr
  /// \param maxIterations
  /// \param distanceThreshold
  /// \return lines
  ///
  docking::LineArray getRansacLines(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudPtr)
  //     std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> RansacLine(typename
  //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudPtr, int maxIterations, float
  //     distanceThreshold) std::pair<pcl::ModelCoefficients::Ptr,
  //     pcl::PointIndices::Ptr> RansacLine(typename
  //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int maxIterations, float
  //     distanceThreshold)
  {
    pcl::PointCloud<pcl::PointXYZRGB> copyInputCloud = *inputCloudPtr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr copyInputCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>(copyInputCloud));
    copyInputCloudPtr->header.seq = inputCloudPtr->header.seq;
    //std::cout << std::endl;
//    ROS_INFO_STREAM("RANSAC Called on Cloud ID " << copyInputCloudPtr->header.seq);
    pcl::ModelCoefficients::Ptr coefficientsPtr(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliersPtr(new pcl::PointIndices());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(RS_max_iter_);
    seg.setDistanceThreshold(RS_dist_thresh_);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> lineVectorPCL;

    docking::LineArray lines;

    std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segResult;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPtr, outCloudPtr;
    inCloudPtr = copyInputCloudPtr;

    pcl::PointCloud<pcl::PointXYZRGB> combinedLinesCloud;

    while (true) {

      if (inCloudPtr->size() <= RS_min_inliers_) {
//        ROS_INFO_STREAM(std::endl << "RANSAC: Less than " << RS_min_inliers_ << " points left");
        break;
      }

      // Segment the current largest line component from the input cloud
      seg.setInputCloud(inCloudPtr);
      seg.segment(*inliersPtr, *coefficientsPtr);

      if (inliersPtr->indices.size() <= RS_min_inliers_) {
//        ROS_INFO_STREAM("RANSAC: Less than " << RS_min_inliers_ << " inliers left");
        break;
      }
//      ROS_INFO_STREAM("RANSAC: Detected Line ID " << lines_.lines.size());
//      ROS_INFO_STREAM("Inlier Points" << inliersPtr->indices << "");
      segResult = SeparateClouds(inliersPtr, inCloudPtr);

      // Create new cloud for detected line
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineCloudPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
      *lineCloudPCLPtr = *segResult.first;
      lineCloudPCLPtr->width = inliersPtr->indices.size();
      lineCloudPCLPtr->height = 1;
      lineCloudPCLPtr->is_dense = true;

//      ROS_INFO_STREAM("RANSAC: Coloring Line at index " << lineVectorPCL.size());
      ColorCloud(lineCloudPCLPtr, lineVectorPCL.size());

      // Add detected line to output cloud and lines array
      lineVectorPCL.push_back(lineCloudPCLPtr);
//      ROS_INFO_STREAM("RANSAC: Added Line to PCLVector at index "<< lineVectorPCL.size() - 1 << " Total lines in Vector: " << lineVectorPCL.size());

      combinedLinesCloud += *lineCloudPCLPtr;

      docking::Line line = rosifyLine(lineCloudPCLPtr, inliersPtr, coefficientsPtr);
//      line.centroid = getCentroid(lineCloudPCLPtr);
//      line.segment = getSegment(lineCloudPCLPtr);
//      line.length.data = getEuclideanDistance(line);
//      updateSegmentList(line);
//      updateLineList(line);
//      markLine(line);

      //        ROS_INFO_STREAM("ROSIFYING LINE-- with " <<
      //        line.points.indices.size() << " points and coefficients " <<
      //        line.coefficients);
//      line =
//      printLineInfo(line);
      line.header = line.cloud.header = header_;
//      ROS_INFO_STREAM("RANSAC: Adding Line " << line.lineID.data << " at index " << lines.lines.size() << " for Cluster ID " << copyInputCloudPtr->header.seq);
      lines.lines.push_back(line);
//      ROS_INFO_STREAM("RANSAC: Total lines in MSG: " << lines.lines.size());

      // Assign cloud of outliers to new input cloud
      inCloudPtr = segResult.second;
//      currentCloudLineID++;
      //std::cout << std::endl;
    }

//    ROS_INFO_STREAM("RANSAC: " << lineVectorPCL.size() << " lines found");

    pcl::toROSMsg(combinedLinesCloud, lines.combinedCloud);

//    ROS_INFO_STREAM("RANSAC RETURNING " << lines.lines.size() << " FOUND LINES ");
    lines.header = lines.combinedCloud.header = header_;
    //std::cout << std::endl;
    return lines;
  }
  ////////////////////////////////// END RANSAC LINE
  /////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////////////////

  ///////////////// BEGIN RANSAC LINE ON CLUSTERS /////////////////
  docking::LineArray getRansacLinesOnCluster(docking::ClusterArray clusters) {
    docking::LineArray lines;

    if(clusters.clusters.size() == 0){
//      ROS_INFO_STREAM("RAN-CLUS-- WARNING: NO CLUSTERS AVAILABLE");
      return lines;
    } else {
//      ROS_INFO_STREAM("RAN-CLUS-- " << clusters.clusters.size() << " AVAILABLE CLUSTERS");
    }

    //        ROS_INFO_STREAM("lines.combinedCloud.frame_id " <<
    //        lines.combinedCloud.header.frame_id);
    //        sensor_msgs::PointCloud2::Ptr linesCombinedCloudPtr (new
    //        sensor_msgs::PointCloud2(lines.combinedCloud));

    lines.header = lines.combinedCloud.header = header_;
    pcl::PointCloud<pcl::PointXYZRGB> linesCombinedPCL;
    int lineID = 0;

    // Iterate through clusters
    for (std::vector<docking::Cluster>::iterator cit = clusters.clusters.begin();
         cit != clusters.clusters.end(); cit++)
    {

//        ROS_INFO_STREAM("RAN-CLUS--RANSACING THROUGH CLUSTER ID " << cit->clusterID.data);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(cit->cloud, *cloudPCLPtr);
        cloudPCLPtr->header.seq = cit->clusterID.data;

        docking::LineArray currentLines = getRansacLines(cloudPCLPtr);
        currentLines.header = currentLines.combinedCloud.header = header_;

//        ROS_INFO_STREAM("RAN-CLUS-- DETECTED " << currentLines.lines.size() << " LINES FROM CLUSTER ID " << cit->clusterID.data);


        for (std::vector<docking::Line>::iterator clit = currentLines.lines.begin();
             clit != currentLines.lines.end(); clit++, lineID++)
        {
//            ROS_INFO_STREAM("RAN-CLUS--ADDING DETECTED LINE ID " << lineID);
//            currentLines.lines.at(lineID).clusterID = cit->clusterID;
            clit->clusterID = cit->clusterID;
//            currentLines.lines.at(lineID).lineID.data = lineID;
//            clit->clusterID = cit->clusterID;
//            lines.lines.push_back(currentLines.lines.at(lineID));
            lines.lines.push_back(*clit);

            cit->lines.lines.push_back(*clit);

        }

//        ROS_INFO_STREAM("RAN-CLUS--COMBINING CLOUDS OF DETECTED LINES FROM CLUSTER ID " << cit->clusterID.data);
        pcl::PointCloud<pcl::PointXYZRGB> currentLinesCloudPCL;
        pcl::fromROSMsg(currentLines.combinedCloud, currentLinesCloudPCL);
        linesCombinedPCL = linesCombinedPCL + currentLinesCloudPCL;


    }

//    ROS_INFO_STREAM("RAN-CLUS--RETURNING ALL DETECTED LINES FROM ALL CLUSTERS");
    pcl::toROSMsg(linesCombinedPCL, lines.combinedCloud);
    lines.header = lines.combinedCloud.header = header_;
//    ROS_INFO_STREAM("lines.combinedCloud.frame_id " << lines.combinedCloud.header.frame_id);
    //std::cout << std::endl;
    return lines;
  }
  ///////////////// END RANSAC LINE ON CLUSTERS /////////////////


  ///////////////// BEGIN LINE TO SEGMENT /////////////////
  /// \brief getSegment
  /// \param inCloudPtr
  /// \return Segment
  ///
  jsk_recognition_msgs::Segment getSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPtr){
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
  geometry_msgs::Pose getCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPtr) {
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
  docking::Line rosifyLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudPCLPtr,
                           pcl::PointIndices::Ptr &indicesPCLPtr,
                           pcl::ModelCoefficients::Ptr &coefficientsPCLPtr) {
    sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);
    pcl_msgs::PointIndicesPtr indicesMsgPtr(new pcl_msgs::PointIndices);
    pcl_msgs::ModelCoefficientsPtr coefficientsMsgPtr(
        new pcl_msgs::ModelCoefficients);

    pcl::toROSMsg(*cloudPCLPtr, *cloudMsgPtr);
    //      pcl_conversions::moveFromPCL(*indicesPCLPtr,*indicesMsgPtr);
    //      pcl_conversions::moveFromPCL(*coefficientsPCLPtr,*coefficientsMsgPtr);

//    ROS_INFO_STREAM("rosifyLine: CONVERTING CURRENT LINE TO ROS MSG");
    docking::Line line;
    line.lineID.data = lines_.lines.size();
    line.cloud = *cloudMsgPtr;
    line.points.indices = indicesPCLPtr->indices;
    line.coefficients.values = coefficientsPCLPtr->values;
//    ROS_INFO_STREAM("rosifyLine: GETTING CENTROID");
    line.centroid = getCentroid(cloudPCLPtr);
//    ROS_INFO_STREAM("rosifyLine: GETTING SEGMENT");
    line.segment = getSegment(cloudPCLPtr);
//    ROS_INFO_STREAM("rosifyLine: GETTING EUCLIDEAN DISTANCE");
    line.length.data = getEuclideanDistance(line);
//    ROS_INFO_STREAM("rosifyLine: UPDATING SEGMENT LIST");
    updateSegmentList(line);
//    ROS_INFO_STREAM("rosifyLine: UPDATING LINE LIST");
    updateLineList(line);
//    ROS_INFO_STREAM("rosifyLine: MARKING LINE");
    markLine(line);

    line.header = header_;
    return line;
  }
  ///////////////// END ROSIFY LINE /////////////////

  ///////////////// BEGIN PUBLISH MARKER /////////////////
//  void publishMarker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudPCLPtr,
//                     pcl::PointIndices::Ptr &indicesPCLPtr,
//                     pcl::ModelCoefficients::Ptr &coefficientsPCLPtr) {
//    sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);
//  }
  ///////////////// END PUBLISH MARKER /////////////////

  ///////////////// BEGIN SEPARATE CLOUDS /////////////////
  /// \brief SeparateClouds
  /// \param inliers
  /// \param cloud
  /// \return
  ///
  std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,pcl::PointCloud<pcl::PointXYZRGB>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointIndices outliers;

    // Obtain the inlier point cloud by copying only inlier indices to the
    // planeCloud for(int index : inliers->indices)
    //     inCloud->points.push_back(cloud->points[index]);
    // ****** ^ does the same as below for the inliers, cuz we already have the
    // inlier indices******
    //Create extract object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract (true);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    // Get plane cloud (inliers)
    extract.setNegative(false); // Extract the inliers, not outliers
    extract.filter(*inCloud);   // Output cloud

    extract.getRemovedIndices(outliers);

    // Get obstacle cloud (outliers)
    extract.setNegative(true); // Extract the outliers, not inliers
    extract.filter(*outCloud); // Output cloud

//    ROS_INFO_STREAM("Inliers: " << inCloud->width * inCloud->height << " points Outliers: " << outCloud->width * outCloud->height << " points");
//    ROS_INFO_STREAM("Inliers: ");
//    printIndices(*inliers);
//    ROS_INFO_STREAM("Outliers: ");
//    printIndices(outliers);

    std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,pcl::PointCloud<pcl::PointXYZRGB>::Ptr>segResult(inCloud, outCloud);
    return segResult;
  }
  ///////////////// END SEPARATE CLOUDS /////////////////

  void printIndices(pcl::PointIndices indices) {
     for(size_t i=0; i < indices.indices.size(); i++)
        std::cout << indices.indices.at(i) << ' ';
     //std::cout << std::endl;
  }

  ///////////////// BEGIN COMBINE CLOUDS /////////////////
  /// \brief CombineClouds      std::vector of pcl::PointXYZRGB Clouds
  /// \param lineCloudVector
  /// \param combinedCloud
  ///
  void CombineClouds(
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> lineCloudVector,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr combinedCloud) {
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

    pcl::PointCloud<pcl::PointXYZRGB> augendCloudPCL;
    //      ROS_INFO_STREAM("CONVERTING AUGENDCLOUDMSG TO PCL");
    pcl::fromROSMsg(augendCloudMsg, augendCloudPCL);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr addendCloudPCLPtr(
        new pcl::PointCloud<pcl::PointXYZRGB>);
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
  void VoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPtr,float leafSize,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloudPtr) {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(inCloudPtr);
    voxel.setLeafSize(Voxel_leaf_size_, Voxel_leaf_size_, Voxel_leaf_size_);
    voxel.filter(*filteredCloudPtr);
  }

  ///////////////// END VOXEL GRID /////////////////




  void printDebugCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr debugCloudPtr) {
    sensor_msgs::PointCloud2 debugCloudMsg;
    pcl::toROSMsg(*debugCloudPtr, debugCloudMsg);
    debugCloudMsg.header = header_;
    debug_pub_.publish(debugCloudMsg);
  }

  void printDebugCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr debugCloudPtr) {
    sensor_msgs::PointCloud2 debugCloudMsg;
    pcl::toROSMsg(*debugCloudPtr, debugCloudMsg);
    debugCloudMsg.header = header_;
    debug_pub_.publish(debugCloudMsg);
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

//       ROS_INFO_STREAM("COMPARING LINES");
//       printLineInfo(l1);
//       printLineInfo(l2);
//       ROS_INFO_STREAM("COMPARING LINES DELTA - Centroid: " << centroidDelta << " Points: " << pointsDelta << " Coefficients: " << coefficientDelta << " Segment: " << segmentDelta);

       totalDelta = centroidDelta + coefficientDelta + segmentDelta + pointsDelta;
       totalDelta = totalDelta/4;

      if (totalDelta <= CL_total_delta_){
        return true;
      } else {
        return false;
      }
    }

    void updateLineList(docking::Line line)
    {
      lines_.header = header_;
//      ROS_INFO_STREAM("UPDATING LINE LIST");
      bool doesExist = false;
      float lineDelta;
      for (size_t i =0; i < lines_.lines.size(); i++){
        doesExist = compareLines(line, lines_.lines.at(i));
//        ROS_INFO_STREAM("COMPARE LINES - DETECTED LINE " << line);
//        ROS_INFO_STREAM("COMPARE LINES - LINE LIST INDEX " << i << " " << lines_.lines.at(i));
//        ROS_INFO_STREAM("UPDATING LINE LIST - DELTA: " << lineDelta);
      }

      if(!doesExist){
//        ROS_INFO_STREAM("UPDATING LINE LIST - LINE IS UNIQUE, ADDING TO LIST");
        lines_.lines.push_back(line);
      } else if (lines_.lines.size() == 0){
//        ROS_INFO_STREAM("UPDATING LINE LIST - LIST IS EMPTY, ADDING TO LIST");
        lines_.lines.push_back(line);
      } else {
//        ROS_INFO_STREAM("UPDATING LINE LIST - LINE ALREADY EXISTS");
      }

//      ROS_INFO_STREAM("UPDATING LINE LIST - TOTAL LINES = " << lines_.lines.size());
      //std::cout << std::endl;
    }

      void updateSegmentList(docking::Line line)
      {
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
//        //std::cout << std::endl;
      }

      bool readPointCloudFile(const std::string& filePath, pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLPtr) {

        if (filePath.find(".ply") != std::string::npos) {
          ROS_INFO_STREAM("readPointCloudFile: PLY file found");
          // Load .ply file.
          if (pcl::io::loadPLYFile(filePath, *PCLPtr) != 0) {
            return false;
          }
          ROS_INFO_STREAM("readPointCloudFile: PLY file loaded");


      } else if (filePath.find(".pcd") != std::string::npos) {
          ROS_INFO_STREAM("readPointCloudFile: PCD file found");
          // Load .pcd file.

          if (pcl::io::loadPCDFile(filePath, *PCLPtr) != 0) {
            return false;
          }
          ROS_INFO_STREAM("readPointCloudFile: PCD file loaded");

        } else {

          ROS_ERROR_STREAM("readPointCloudFile: Data format not supported.");
          std::cout << "readPointCloudFile: FAILED CLOUD FILE PATH: " << filePath << std::endl;
          return false;
        }

        ROS_INFO_STREAM("readPointCloudFile: SUCCESSFULLY Loaded point cloud with " << PCLPtr->height * PCLPtr->width << " points.");
        return true;
      }

      ///////////////// BEGIN ICP2D /////////////////

      bool ICP2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPCLPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloudPtr,pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registrationPtr) {

        // ICP object.
        // pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> registration;
        Eigen::Matrix4f t (Eigen::Matrix4f::Identity ());

        pcl::registration::WarpPointRigid3D<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr warp_fcn
              (new pcl::registration::WarpPointRigid3D<pcl::PointXYZRGB, pcl::PointXYZRGB>);

        // Create a TransformationEstimationLM object, and set the warp to it
        pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr te (new pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB, pcl::PointXYZRGB>);
        te->setWarpFunction (warp_fcn);

        // Pass the TransformationEstimation objec to the ICP algorithm
        registrationPtr->setTransformationEstimation (te);

        registrationPtr->setMaximumIterations (100);
//        registrationPtr->setMaxCorrespondenceDistance (0.05);
//        registrationPtr->setRANSACOutlierRejectionThreshold (0.05);

//        ROS_INFO_STREAM("ICP2D--ASSIGNING CLOUD POINTERS");
//        registrationPtr->setInputSource(targetPCLPtr);
//        registrationPtr->setInputTarget(inputCloudPtr);
        registrationPtr->setInputSource(inputCloudPtr);
        registrationPtr->setInputTarget(targetPCLPtr);
//        ROS_INFO_STREAM("ICP2D--ALIGNING CLOUDS");
        registrationPtr->align(*outCloudPtr);
        ROS_INFO_STREAM("ICP2D--CHECKING CONVERGENCE");
        if (registrationPtr->hasConverged())
        {
          t *= registrationPtr->getFinalTransformation ();
          std::cout << "ICP2D converged." << std::endl
                << "The score is " << registrationPtr->getFitnessScore() << std::endl;
          std::cout << "Transformation matrix:" << std::endl;
          std::cout << registrationPtr->getFinalTransformation() << std::endl;
          return true;
        }
        else
        {
          std::cout << "ICP2D did not converge." << std::endl;
          return false;
        }

      }
      ///////////////// END ICP2D /////////////////


      ///////////////// BEGIN ICP /////////////////

      bool ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPCLPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloudPtr,pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registrationPtr) {

        // ICP object.
//        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> registration;
//        ROS_INFO_STREAM("ICP--ASSIGNING CLOUD POINTERS");
        registrationPtr->setInputSource(inputCloudPtr);
        registrationPtr->setInputTarget(targetPCLPtr);
//        ROS_INFO_STREAM("ICP--ALIGNING CLOUDS");
        registrationPtr->align(*outCloudPtr);
//        ROS_INFO_STREAM("ICP--CHECKING CONVERGENCE");
        if (registrationPtr->hasConverged())
        {
//          std::cout << "ICP converged." << std::endl
//                << "The score is " << registrationPtr->getFitnessScore() << std::endl;
//          std::cout << "Transformation matrix:" << std::endl;
//          std::cout << registrationPtr->getFinalTransformation() << std::endl;
          return true;
        }
        else
        {
          std::cout << "ICP did not converge." << std::endl;
          return false;
        }

      }
      ///////////////// END ICP /////////////////

      ///////////////// BEGIN clusterICP /////////////////
      void clusterICP(docking::Cluster::Ptr clusterPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPCLPtr){

//          ROS_INFO_STREAM("ICP-CLUS--PROCESSING CLUSTER ID " << clusterPtr->clusterID.data);

          pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloudPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
          pcl::fromROSMsg(clusterPtr->cloud, *inCloudPCLPtr);
//          ROS_INFO_STREAM("ICP-CLUS--CREATING ICP REGISTRATION OBJECT ");

          // ICP object.
          pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registrationPtr (new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());
          bool success = ICP(inCloudPCLPtr, targetPCLPtr, outCloudPCLPtr, registrationPtr);

//          pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registrationPtr (new pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB>());
//          bool success = ICP2D(inCloudPCLPtr, targetPCLPtr, outCloudPCLPtr, registrationPtr);

//          ROS_INFO_STREAM("ICP-CLUS--PERFORMING ICP ON CLUSTER ID " << clusterPtr->clusterID.data);

          if(success){
//            ROS_INFO_STREAM("ICP-CLUS-- SUCCESS ICP ON CLUSTER ID " << clusterPtr->clusterID.data);
            sensor_msgs::PointCloud2 ICPCombinedCloud;
            pcl::toROSMsg(*outCloudPCLPtr,ICPCombinedCloud);
            clusterPtr->icpCombinedCloud = ICPCombinedCloud;
            ROS_INFO_STREAM("ICP-CLUS-- GETTING FITNESS SCORE ICP ON CLUSTER ID " << clusterPtr->clusterID.data << " " << registrationPtr->getFitnessScore());
            clusterPtr->icp.score = registrationPtr->getFitnessScore();

            Eigen::Matrix4f transformation = registrationPtr->final_transformation_;
//            ROS_INFO_STREAM("ICP-CLUS-- GETTING final_transformation_ ICP ON CLUSTER ID " << transformation);

            transformation = registrationPtr->getFinalTransformation();
            ROS_INFO_STREAM("ICP-CLUS-- GETTING getFinalTransformation ICP ON CLUSTER ID " << clusterPtr->clusterID.data << "\n" << transformation);

            clusterPtr->icp.poseStamped.pose = Matrix4TFtoPose(transformation);
            clusterPtr->icp.transformStamped = Matrix4TFtoTransform(transformation);
            clusterPtr->icp.poseStamped.header = header_;

//            ROS_INFO_STREAM("ICP-CLUS-- CLUSTER ID " << clusterPtr->clusterID.data << " POSE: " << clusterPtr->icp.poseStamped);
//            ROS_INFO_STREAM("ICP-CLUS-- CLUSTER ID " << clusterPtr->clusterID.data << " TRANSFORM: " << clusterPtr->icp.transformStamped);
//            std::cout << std::endl;
//            ROS_INFO_STREAM("ICP-CLUS-- CLUSTER ID " << clusterPtr->clusterID.data << " ICP SCORE: " << clusterPtr->icp.score);
//            ROS_INFO_STREAM("ICP-CLUS-- ICP THRESHOLD SCORE: " << icpScore_);
            if(clusterPtr->icp.score < icpScore_){
                clusterPtr->isDock.data = true;
//                ROS_WARN_STREAM("ICP-CLUS-- CLUSTER ID " << clusterPtr->clusterID.data << " SUCCESSFULLY IDENTIFIED AS DOCK!!!!!!!!!!");
            }
          } else {
            ROS_ERROR_STREAM("ICP-CLUS-- FAILED ICP ON CLUSTER ID " << clusterPtr->clusterID.data);
          }

      }
      ///////////////// END clusterICP /////////////////

      ///////////////// BEGIN clusterArrayICP /////////////////
      bool clusterArrayICP(docking::ClusterArray::Ptr clustersPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPCLPtr, docking::Cluster::Ptr dockClusterPtr)
      {
//      ROS_INFO_STREAM("ICP-CLUS-ARRAY--BEGINNING ICP ON CLUSTERS ");


//        for (std::vector<docking::Cluster>::iterator cit = clustersPtr->clusters.begin();
//             cit != clustersPtr->clusters.end(); cit++)
          for(size_t i=0; i<clustersPtr->clusters.size();i++)
        {
          docking::Cluster::Ptr currentClusterPtr(new docking::Cluster());
          *currentClusterPtr = clustersPtr->clusters.at(i);
          clusterICP(currentClusterPtr, targetPCLPtr);
          clustersPtr->clusters.at(i) = *currentClusterPtr;
          if(currentClusterPtr->isDock.data){
//            ROS_WARN_STREAM("ICP-CLUS-ARRAY--DOCK POTENTIALLY IDENTIFIED ");
            *dockClusterPtr = clustersPtr->clusters.at(i);
          }

        }

//      ROS_INFO_STREAM("ICP-CLUS-ARRAY--COMPLETED PERFORMING ICP ON CLUSTERS ");
//      ROS_INFO_STREAM("ICP-CLUS-ARRAY--dockClusterPtr->isDock.data = " << dockClusterPtr->isDock.data);

      if(dockClusterPtr->isDock.data){
//        ROS_WARN_STREAM("ICP-CLUS-ARRAY--DOCK SUCCESSFULLY IDENTIFIED ");
        return true;
      }
//      ROS_WARN_STREAM("ICP-CLUS-ARRAY--UNABLE TO IDENTIFY DOCK");
      return false;
      }
      ///////////////// END clusterArrayICP /////////////////


};

//  template class SegmentLineNode<pcl::PointXYZ>;
//  template class SegmentLineNode<pcl::PointXYZI>;
//template class SegmentLineNode<pcl::PointXYZRGB>;
//  template class SegmentLineNode<pcl::PointXYZRGBA>;

//} // namespace docking

//#include <docking/impl/SegmentLineNode.hpp>
//#define PCL_INSTANTIATE_SegmentLineNode(T) template class PCL_EXPORTS
// pcl::SegmentLineNode<T>;

#endif /*"SEGMENTLINENODE_H_"*/
