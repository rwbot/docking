// -*- mode: c++ -*-
#ifndef DETECTIONNODE_H_
#define DETECTIONNODE_H_

#include <docking/Headers.h>
#include <docking/DetectionNodeConfig.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> POINT;

//namespace docking {

class DetectionNode {
///////////////// BEGIN VARIABLES /////////////////
public:
  DetectionNode(ros::NodeHandle nh) :
    nh_(nh), tfListener_(tfBuffer_) {
    startDynamicReconfigureServer();
    initParams();
    startPub();
    activationSub();
    initGlobals();
    initDockParams();
    ros::Duration(1).sleep(); // Wait for pointcloud_to_laserscan node to init and publish cloud topic
    startCloudSub(cloud_topic_);
  }
  ~DetectionNode() {}

  ros::Publisher status_pub_;
  ros::Publisher clusters_cloud_pub_;
  ros::Publisher clusters_pub_;
  ros::Publisher lines_cloud_pub_;
  ros::Publisher lines_pub_;
  ros::Publisher line_marker_pub_;
  ros::Publisher line_segment_pub_;
  ros::Publisher dock_marker_pub_;
  ros::Publisher dock_pose_marker_pub_;
  ros::Publisher dock_pose_pub_;
  ros::Publisher bbox_pub_;     // Bounding box publisher
  ros::Publisher debug_pub_;    // debug publisher
  ros::Publisher icp_in_pub_;   // ICP Input Cloud publisher
  ros::Publisher icp_target_pub_; // ICP Target Cloud publisher
  ros::Publisher icp_out_pub_;   // ICP Output Cloud publisher
  ros::Subscriber cloudSub_;
  ros::Subscriber activationSub_;
  ros::NodeHandle nh_;
  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<docking::DetectionNodeConfig> dr_srv_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;

  //! Target Dock PCL Cloud Pointer
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dockTargetPCLPtr_;


  //! Global list of line markers for publisher
  visualization_msgs::Marker lines_marker_;
  //! Global list of line msgs for publisher
  docking::LineArray lines_;
  //! Global list of cluster msgs for publisher
  docking::LineArray::Ptr linesPtr_;
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
  std_msgs::Bool found_dock_;
  //! Bool of detection node status
  std_msgs::Bool perform_detection_;
  //! Bool of having printed detection node status
  bool printed_deactivation_status_;

  //! Dock Wing Length
  double dock_wing_length;

  //! Dock target template filepath
  std::string dockFilePath_;
  bool dock_template_loaded_;
  //! ICP Fitness Score Threshold
  double ICP_min_score_;
  //! ICP The maximum distance threshold between two correspondent points
  double ICP_max_correspondence_distance_;
  //! ICP Max iterations
  double ICP_max_iterations_;
  //! ICP Max transformation translation epsilon (diff) btwn previous & current estimated
  double ICP_max_transformation_eps_;
  //! ICP Max transformation rotation epsilon (diff) btwn previous & current estimated
  double ICP_max_transformation_rotation_eps_;
  //! ICP Max Euclidean squared errors
  double ICP_max_euclidean_fitness_eps_;

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
    ROS_INFO_STREAM("startDynamicReconfigureServer: STARTING DYNAMIC RECONFIGURE SERVER");
    dynamic_reconfigure::Server<docking::DetectionNodeConfig>::CallbackType cb;
    cb = boost::bind(&DetectionNode::configCallback, this, _1, _2);
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
//    nh_.param("ICP_min_score", ICP_min_score_, ICP_min_score_);
//    nh_.param("ICP_max_corres_dist", ICP_max_correspondence_distance_, ICP_max_correspondence_distance_);
//    nh_.param("ICP_max_iter", ICP_max_iterations_, ICP_max_iterations_);
//    nh_.param("ICP_max_transl_eps", ICP_max_transformation_eps_, ICP_max_transformation_eps_);
//    nh_.param("ICP_max_rot_eps", ICP_max_transformation_rotation_eps_, ICP_max_transformation_rotation_eps_);
//    nh_.param("ICP_max_eucl_fit_eps", ICP_max_euclidean_fitness_eps_, ICP_max_euclidean_fitness_eps_);

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
  void configCallback(docking::DetectionNodeConfig &config,
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

    ICP_min_score_ = config.ICP_min_score;
    ICP_max_correspondence_distance_ = config.ICP_max_corres_dist;
    ICP_max_iterations_ = config.ICP_max_iter;
    ICP_max_transformation_eps_ = config.ICP_max_transl_eps;
    ICP_max_transformation_rotation_eps_ = config.ICP_max_rot_eps;
    ICP_max_euclidean_fitness_eps_ = config.ICP_max_eucl_fit_eps;

    ROS_INFO_STREAM("configCallback:  Iterations " << ICP_max_iterations_);
    ROS_INFO_STREAM("configCallback:  Max Correspondence Distance " << ICP_max_correspondence_distance_);
    ROS_INFO_STREAM("configCallback:  Max Transformation Epsilon " << ICP_max_transformation_eps_);
    ROS_INFO_STREAM("configCallback:  Max Euclidean Fitness Epsilon " << ICP_max_euclidean_fitness_eps_);


    if ((config.cloud_topic != "") && (config.cloud_topic != cloud_topic_)) {
      cloud_topic_ = config.cloud_topic;
      ROS_INFO_STREAM("configCallback: New Input Cloud Topic");
      startCloudSub(cloud_topic_);
    }

//    if (config.laser_topic != laser_topic_) {
//      laser_topic_ = config.laser_topic;
//      ROS_INFO_STREAM("configCallback: NEW Input Laser Topic" << laser_topic_);
//    }


    // if(dockFilePath_==""){
    //   std::cout << "configCallback: dockFilePath FILEPATH NOT SPECIFIED IN DYNAMIC RECONFIGURE " << dockFilePath_ << std::endl;
    // }
    // else if (config.dock_filepath != dockFilePath_) {
    //   dockFilePath_ = config.dock_filepath;
    //   ROS_INFO_STREAM("configCallback: New Dock Target File Specified");
    //   std::cout << "configCallback: CONFIG FILEPATH: " << config.dock_filepath << std::endl;
    //   std::cout << "configCallback: DOCK TARGET FILEPATH: " << dockFilePath_ << std::endl;
    //   ROS_INFO_STREAM("configCallback: LOADING DOCK TARGET CLOUD FILE");
    //   if(readPointCloudFile(dockFilePath_,dockTargetPCLPtr_) == false){
    //     ROS_ERROR_STREAM("configCallback: FAILED TO LOAD TARGET DOCK FILE");
    //   } else {
    //     ROS_WARN_STREAM("configCallback: SUCCESSFULLY LOADED TARGET DOCK FILE");
    //   }

    // }

  }

  void initGlobals(){
    found_dock_.data = false;
    docking::ClusterArray temp1 = docking::ClusterArray();
    docking::ClusterArray::Ptr tempPtr (new docking::ClusterArray ());
    clustersPtr_ = tempPtr;
    *clustersPtr_ = temp1;

    docking::LineArray temp2 = docking::LineArray();
    docking::LineArray::Ptr tempPtr2 (new docking::LineArray ());
    linesPtr_ = tempPtr2;
    *linesPtr_ = temp2;
  }

  void startPub() {
    status_pub_ = nh_.advertise<std_msgs::Bool>("docking/found_dock", 1);

    clusters_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("docking/clustersCloud", 1);
    clusters_pub_ = nh_.advertise<docking::ClusterArray>("docking/clusters", 1);

    lines_cloud_pub_ =  nh_.advertise<sensor_msgs::PointCloud2>("docking/linesCloud", 1);
    lines_pub_ = nh_.advertise<docking::LineArray>("docking/lines", 1);
    line_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("docking/lines_marker", 1);

    dock_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("docking/dock_marker", 1);
    dock_pose_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("docking/dock_pose_marker", 1);
    dock_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("docking/dock_pose", 1);
    bbox_pub_ = nh_.advertise<docking::BoundingBox>("docking/bbox", 1);

    debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("docking/debugCloud", 1);

    icp_in_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("docking/icp_in_pub", 1);
    icp_target_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("docking/icp_target_pub", 1);
    icp_out_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("docking/icp_out_pub", 1);
  }

  void startCloudSub(std::string cloud_topic) {
    //  std::string ns_cloud_topic = "/" + cloud_topic;
    // if(!checkTopicExists(cloud_topic)){
    //   // ROS_WARN_STREAM("Namespaced Topic " + ns_cloud_topic + " does not exist");
    //   ROS_WARN_STREAM("Topic " + cloud_topic + " does not exist");
    //   ROS_WARN_STREAM("Check to see if the topic is correct or if it is namespaced to continue");
    //   return;
    // }
    ROS_INFO_STREAM("Subscribing to new cloud topic " + cloud_topic_);
    cloudSub_ = nh_.subscribe(cloud_topic, 1, &DetectionNode::cloudCallback, this);
    if(cloudSub_){
      ROS_INFO_STREAM("SUCCESSFULLY subscribed to new cloud topic " + cloud_topic);
    } else {
      ROS_WARN_STREAM("FAILED to subscribe to new cloud topic " + cloud_topic);
    }
  }

  bool checkTopicExists(std::string &topic){
    ROS_INFO_STREAM("Checking existence of new cloud topic " + topic);
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    for (int i=0; i < topic_infos.size(); i++){
     ROS_INFO_STREAM("Check topic #" << i << "  " << topic_infos.at(i).name);
      // if(topic == topic_infos.at(i).name)
      if(topic_infos.at(i).name.find(topic) != std::string::npos)
      {
        return true;
      }
    }
    return false;
  }

  void activationSub(){
    // activationSub_ = nh_.subscribe("docking/perform_detection", 1,
    // &DetectionNode::activationCallback, this);
    activationSub_ = nh_.subscribe("docking/perform_detection", 1, &DetectionNode::activationCallback, this);
  }

  void clearGlobals(){
    lines_.lines.clear();
    lines_marker_.points.clear();
    lines_marker_.colors.clear();
    clustersPtr_->clusters.clear();
    linesPtr_->lines.clear();
//    linesPtr_->segments.clear();
  }


  void activationCallback(const std_msgs::BoolConstPtr &msg){
    if(perform_detection_.data == msg->data){
      return;
    }
    perform_detection_ = *msg;
    ROS_WARN_STREAM("SETTING DETECTION ACTIVATION STATUS TO  " << perform_detection_);
    printed_deactivation_status_ = false;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {

    // Only perform detection if activated
    if(perform_detection_.data == false){
      if(!printed_deactivation_status_){
        ROS_WARN_STREAM("cloudCallback: DETECTION NOT ACTIVATED");
        printed_deactivation_status_ = true;
      }
      return;
    }

    ros::Time beginCallback = ros::Time::now();

    // ROS_INFO_STREAM("CLOUD CALLBACK: CALLBACK CALLED ");

    if(msg->width == 0 || msg->row_step == 0){
      ROS_WARN_STREAM("CALLBACK: POINT CLOUD MSG EMPTY ");
      return;
    }


   clearGlobals();

    header_ = msg->header;
    linesPtr_->header = clustersPtr_->header = clusters_.header = lines_.header = lines_marker_.header = header_;

  static tf2_ros::TransformBroadcaster tfbr;   


    // Container for original & filtered data
    /*
     * CONVERT POINTCLOUD ROS->PCL
     */
    pcl::PointCloud<pcl::PointXYZRGB> cloudPCL;
    pcl::fromROSMsg(*msg, cloudPCL);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCLPtr(new pcl::PointCloud<pcl::PointXYZRGB>(cloudPCL));
    if(cloudPCLPtr->size() == 0){
      ROS_WARN_STREAM("CALLBACK: PCL POINT CLOUD EMPTY ");
      return;
    }

    /* ========================================
     * VOXEL DOWNSAMPLING
     * ========================================*/
    VoxelGrid(cloudPCLPtr,Voxel_leaf_size_,cloudPCLPtr);

    /* ========================================
     * CLUSTERING
     * ========================================*/
//    clusters_ = this->ClusterPoints(cloudPCLPtr, EC_cluster_tolerance_, EC_min_size_, EC_max_size_);

//    ROS_INFO_STREAM("CLOUD CALLBACK: clustersPtr_.header " << clustersPtr_->header);

    Clustering clustering;
    clustering.setHeader(header_);
    clustering.ClusterPoints(cloudPCLPtr, clustersPtr_, EC_cluster_tolerance_, EC_min_size_, EC_max_size_);
    if(clustersPtr_->clusters.size() == 0){
      ROS_WARN_STREAM("CALLBACK: NO CLUSTERS FOUND ");
      return;
    }

  //  ROS_INFO_STREAM("CLOUD CALLBACK: CLUSTERING RETURNED " << clustersPtr_->clusters.size() << " CLUSTERS");

//    ROS_INFO_STREAM("CLOUD CALLBACK: clusters.combinedCloud.frame_id " << clustersPtr_->combinedCloud.header.frame_id);

    /* ========================================
     * RANSAC LINES
     * ========================================*/

//    ROS_INFO_STREAM("CLOUD CALLBACK: Distance Threshold: "<< RS_dist_thresh_ << " Max Iterations: " << RS_max_iter_);
//    docking::LineArray lines;
    LineDetection lineDetection;
    lineDetection.setHeader(header_);
    lineDetection.setParams(RS_max_iter_, RS_min_inliers_, RS_dist_thresh_);

      /* ========================================
       * RANSAC LINES FROM CLUSTERS
       * ========================================*/
//     ROS_INFO_STREAM("CLOUD CALLBACK: CALLING RANSAC ON CLUSTERED CLOUD ");
    if(clustersPtr_->clusters.size() > 0){
      lineDetection.getRansacLinesOnCluster(clustersPtr_, linesPtr_);
    }


//   ROS_INFO_STREAM("CLOUD CALLBACK: RAN-CLUS-- COMPLETE");

    /* ========================================
     * PUBLISH CLOUD
     * ========================================*/

    if(clustersPtr_->clusters.size()>0){
      clusters_cloud_pub_.publish(clustersPtr_->combinedCloud);
      clusters_pub_.publish(clustersPtr_);
      lines_cloud_pub_.publish(linesPtr_->combinedCloud);
      //          docking::LineArray::Ptr linesPtr (new docking::LineArray(lines));
      lines_pub_.publish(linesPtr_);

      // Assign Dock Cluster When Debugging in Dock ONly World
//      docking::Cluster dockCluster = clustersPtr_->clusters.front();

//      dock_marker_pub_.publish(dockCluster.bbox.marker);
//      bbox_pub_.publish(dockCluster.bbox);
    }

//    if(clustersPtr_->clusters.size()>0)


    //          ROS_INFO_STREAM("CALLBACK: Publishing cluster with centroid " <<
    //          pubCluster.centroid);



    /* ========================================
     * ITERATIVE CLOSEST POINT
     * ========================================*/

//    ROS_INFO_STREAM("CLOUD CALLBACK: BEGINNING ITERATIVE CLOSEST POINT");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ICPInputCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ICPOutCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    docking::Cluster::Ptr dockClusterPtr(new docking::Cluster());
    PoseEstimation poseEstimation;
    poseEstimation.setHeader(header_);
    poseEstimation.setICPScore(ICP_min_score_);
    poseEstimation.setMaxIterations(ICP_max_iterations_);
    poseEstimation.setMaxCorrespondenceDistance(ICP_max_correspondence_distance_);
    poseEstimation.setMaxTransformationEps(ICP_max_transformation_eps_);
    poseEstimation.setMaxEuclideanFitnessEps(ICP_max_euclidean_fitness_eps_);
    bool icpSuccess = poseEstimation.clusterArrayICP(clustersPtr_, dockTargetPCLPtr_,dockClusterPtr);
//    ROS_INFO_STREAM("CLOUD CALLBACK: ICP FINISHED");


    ICPInputCloudPtr->header.frame_id = header_.frame_id;
    dockTargetPCLPtr_->header.frame_id = header_.frame_id;
    ICPOutCloudPtr->header.frame_id = header_.frame_id;

    if(icpSuccess){
      // ROS_INFO_STREAM("CLOUD CALLBACK: ICP SUCCESSFUL ");
      found_dock_.data = true;

      icp_in_pub_.publish(dockClusterPtr->cloud);
      icp_out_pub_.publish(dockClusterPtr->icpCombinedCloud);
      dock_pose_pub_.publish(dockClusterPtr->icp.poseStamped);
      dock_pose_marker_pub_.publish(dockClusterPtr->icp.poseTextMarker);
//      dock_marker_pub_.publish(dockClusterPtr->bbox.marker);
      dock_marker_pub_.publish(dockClusterPtr->bbox.marker);
      bbox_pub_.publish(dockClusterPtr->bbox);

      dockClusterPtr->icp.transformStamped.header.stamp = ros::Time::now();
      dockClusterPtr->icp.transformStamped.header.frame_id = msg->header.frame_id;
      dockClusterPtr->icp.transformStamped.child_frame_id = "dock";
//      tfbr.sendTransform(dockClusterPtr->icp.transformStamped);
      tfBroadcaster_.sendTransform(dockClusterPtr->icp.transformStamped);
      icp_target_pub_.publish(dockTargetPCLPtr_);     
    } else {
      ROS_WARN_STREAM("CLOUD CALLBACK: ICP FAILED ");
      found_dock_.data = false;
    }

    ////////////////////////////////////////////////////////////////////
    // TRANSFORM
    ////////////////////////////////////////////////////////////////////
    /// \brief ROS_INFO_STREAM
    /////    tfBuffer_.transform(targetPose,base2TargetPose,robotFrameID);
    //    sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);
//        geometry_msgs::TransformStamped laser_to_base_link;
//        sensor_msgs::PointCloud2::Ptr transformed_cloud (new sensor_msgs::PointCloud2());
//        *transformed_cloud = *msg;


//    //    ROS_INFO_STREAM("CLOUD CALLBACK: ORIGINAL SENSOR_MSGS::POINTCLOUD2" << *transformed_cloud);
//        try {
//          laser_to_base_link = tfBuffer_.lookupTransform(msg->header.frame_id, "base_link", ros::Time(0), ros::Duration(1.0));
//    //      tfBuffer_.transform(msg,transformed_cloud,"base_link");
//          pcl_ros::transformPointCloud("base_link", laser_to_base_link.transform, *msg,  *transformed_cloud);
//    //      ROS_INFO_STREAM("CLOUD CALLBACK: TRANSFORMED SENSOR_MSGS::POINTCLOUD2" << *transformed_cloud);

//        }
//        catch (tf::TransformException ex) {
//          ROS_ERROR("%s",ex.what());
//        }

//        debug_pub_.publish(transformed_cloud);

    status_pub_.publish(found_dock_);

    // ROS_INFO_STREAM("CLOUD CALLBACK: CALLBACK COMPLETE");
    ros::Duration total = ros::Time::now() - beginCallback;
    ROS_INFO_STREAM("DETECTION TOOK " << total.toSec() << " secs");
    std::cout << std::endl;
  }

//  ///////////////// BEGIN ROSIFY LINE /////////////////
//  docking::Line rosifyLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudPCLPtr,
//                           pcl::PointIndices::Ptr &indicesPCLPtr,
//                           pcl::ModelCoefficients::Ptr &coefficientsPCLPtr) {
//    sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);
//    pcl_msgs::PointIndicesPtr indicesMsgPtr(new pcl_msgs::PointIndices);
//    pcl_msgs::ModelCoefficientsPtr coefficientsMsgPtr(
//        new pcl_msgs::ModelCoefficients);

//    pcl::toROSMsg(*cloudPCLPtr, *cloudMsgPtr);
//    //      pcl_conversions::moveFromPCL(*indicesPCLPtr,*indicesMsgPtr);
//    //      pcl_conversions::moveFromPCL(*coefficientsPCLPtr,*coefficientsMsgPtr);

////    ROS_INFO_STREAM("rosifyLine: CONVERTING CURRENT LINE TO ROS MSG");
//    docking::Line line;
//    line.lineID.data = lines_.lines.size();
//    line.cloud = *cloudMsgPtr;
//    line.points.indices = indicesPCLPtr->indices;
//    line.coefficients.values = coefficientsPCLPtr->values;
////    ROS_INFO_STREAM("rosifyLine: GETTING CENTROID");
//    line.centroid = getCentroid(cloudPCLPtr);
////    ROS_INFO_STREAM("rosifyLine: GETTING SEGMENT");
//    line.segment = getSegment(cloudPCLPtr);
////    ROS_INFO_STREAM("rosifyLine: GETTING EUCLIDEAN DISTANCE");
//    line.length.data = getEuclideanDistance(line);
////    ROS_INFO_STREAM("rosifyLine: UPDATING LINE LIST");
//    updateLineList(line);
////    ROS_INFO_STREAM("rosifyLine: MARKING LINE");
//    markLine(line);

//    line.header = header_;
//    return line;
//  }
//  ///////////////// END ROSIFY LINE /////////////////

  ///////////////// BEGIN PUBLISH MARKER /////////////////
//  void publishMarker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudPCLPtr,
//                     pcl::PointIndices::Ptr &indicesPCLPtr,
//                     pcl::ModelCoefficients::Ptr &coefficientsPCLPtr) {
//    sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);
//  }
  ///////////////// END PUBLISH MARKER /////////////////

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
    voxel.setLeafSize(leafSize, leafSize, leafSize);
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

//  ///////////////// BEGIN MARK LINE /////////////////
//  //    visualization_msgs::Marker
//  //    mark_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster,
//  //    std::string ns ,int id, float r, float g, float b)
//  void markLine(docking::Line line) {

//    lines_marker_.type = visualization_msgs::Marker::LINE_LIST;
//    lines_marker_.header = header_;
//    lines_marker_.ns = "docking";
//    lines_marker_.id = 0;

//    lines_marker_.action = visualization_msgs::Marker::ADD;

////    lines_marker_.pose = line.centroid;
//    lines_marker_.pose.orientation.w = 1.0;
//    lines_marker_.points.push_back(line.segment.start_point);
//    lines_marker_.points.push_back(line.segment.end_point);
//    lines_marker_.scale.x = lines_marker_.scale.y = lines_marker_.scale.z = 0.05;

//    lines_marker_.color.r = 0.0f;
//    lines_marker_.color.g = 1.0f;
//    lines_marker_.color.b = 0.0f;
//    lines_marker_.color.a = 0.15f;

////    std_msgs::ColorRGBA color = lines_marker_.color;
//    lines_marker_.colors.push_back(lines_marker_.color);
//    lines_marker_.colors.push_back(lines_marker_.color);

//    lines_marker_.lifetime = ros::Duration();
//    //       marker.lifetime = ros::Duration(0.5);
//  }

    bool compareLines(docking::Line l1, docking::Line l2, float CL_total_delta){
      float totalDelta;

      float centroidDelta = comparePoses(l1.centroid, l2.centroid);
      float coefficientDelta = compareCoefficients(l1.coefficients, l2.coefficients);
//      float segmentDelta = compareSegments(l1.segment, l2.segment);
      float pointsDelta = comparePointIndices(l1.points, l2.points);

//       ROS_INFO_STREAM("COMPARING LINES");
//       printLineInfo(l1);
//       printLineInfo(l2);
//       ROS_INFO_STREAM("COMPARING LINES DELTA - Centroid: " << centroidDelta << " Points: " << pointsDelta << " Coefficients: " << coefficientDelta << " Segment: " << segmentDelta);

       totalDelta = centroidDelta + coefficientDelta /*+ segmentDelta*/ + pointsDelta;
       totalDelta = totalDelta/4;

      if (totalDelta <= CL_total_delta){
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
        doesExist = compareLines(line, lines_.lines.at(i), CL_total_delta_);
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

};

//} // namespace docking

#endif /*"DETECTIONNODE_H_"*/
