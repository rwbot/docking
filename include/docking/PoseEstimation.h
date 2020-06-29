#ifndef ICP_H
#define ICP_H

#include <docking/Headers.h>

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

class PoseEstimation {

public:
  PoseEstimation(){}
  ~PoseEstimation(){}

  std_msgs::Header header_;
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

  void setHeader(std_msgs::Header header){    header_ = header;  }
  void setICPScore(double ICP_min_score){    ICP_min_score_ = ICP_min_score;  }
  void setMaxCorrespondenceDistance(double ICP_max_correspondence_distance){    ICP_max_correspondence_distance_ = ICP_max_correspondence_distance;  }
  void setMaxIterations(double ICP_max_iterations){    ICP_max_iterations_ = ICP_max_iterations;  }
  void setMaxTransformationEps(double ICP_max_transformation_eps){    ICP_max_transformation_eps_ = ICP_max_transformation_eps;  }
  void setMaxRotationEps(double ICP_max_transformation_rotation_eps){    ICP_max_transformation_rotation_eps_ = ICP_max_transformation_rotation_eps; }
  void setMaxEuclideanFitnessEps(double ICP_max_euclidean_fitness_eps){    ICP_max_euclidean_fitness_eps_ = ICP_max_euclidean_fitness_eps;  }


  void setParams(pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registrationPtr){
    registrationPtr->setMaximumIterations (ICP_max_iterations_);
    registrationPtr->setMaxCorrespondenceDistance (ICP_max_correspondence_distance_);
    registrationPtr->setTransformationEpsilon(ICP_max_transformation_eps_);
    registrationPtr->setEuclideanFitnessEpsilon(ICP_max_euclidean_fitness_eps_);
    ROS_INFO_STREAM("ICP-SETPARAMS-- GET Iterations " << registrationPtr->getMaximumIterations());
    ROS_INFO_STREAM("ICP-SETPARAMS-- GET Max Correspondence Distance " << registrationPtr->getMaxCorrespondenceDistance());
    ROS_INFO_STREAM("ICP-SETPARAMS-- GET Max Transformation Epsilon " << registrationPtr->getTransformationEpsilon());
    ROS_INFO_STREAM("ICP-SETPARAMS-- GET Max Euclidean Fitness Epsilon " << registrationPtr->getEuclideanFitnessEpsilon());
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

//    registrationPtr->setRANSACOutlierRejectionThreshold (0.05);

//        ROS_INFO_STREAM("ICP2D--ASSIGNING CLOUD POINTERS");
    registrationPtr->setInputSource(inputCloudPtr);
    registrationPtr->setInputTarget(targetPCLPtr);
//        ROS_INFO_STREAM("ICP2D--ALIGNING CLOUDS");
    registrationPtr->align(*outCloudPtr);
//    ROS_INFO_STREAM("ICP2D--CHECKING CONVERGENCE");
    if (registrationPtr->hasConverged())
    {
//      std::cout << "ICP2D converged." << std::endl
//            << "The score is " << registrationPtr->getFitnessScore() << std::endl;
      // ROS_INFO_STREAM("ICP2D--ICP2D converged with score " << registrationPtr->getFitnessScore());
//      std::cout << "Transformation matrix:" << std::endl;
//      std::cout << registrationPtr->getFinalTransformation() << std::endl;
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
      bool success;



      // ICP object.
//      pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registrationPtr (new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());

      pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registrationPtr (new pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB>());


      registrationPtr->setMaximumIterations (ICP_max_iterations_);
//      registrationPtr->setMaxCorrespondenceDistance (ICP_max_correspondence_distance_);
//      registrationPtr->setTransformationEpsilon(ICP_max_transformation_eps_);
//      registrationPtr->setEuclideanFitnessEpsilon(ICP_max_euclidean_fitness_eps_);

//      ROS_INFO_STREAM("ICP-CLUS-- GET Iterations " << registrationPtr->getMaximumIterations());
//      ROS_INFO_STREAM("ICP-CLUS-- GET Max Correspondence Distance " << registrationPtr->getMaxCorrespondenceDistance());
//      ROS_INFO_STREAM("ICP-CLUS-- GET Max Transformation Epsilon " << registrationPtr->getTransformationEpsilon());
//      ROS_INFO_STREAM("ICP-CLUS-- GET Max Euclidean Fitness Epsilon " << registrationPtr->getEuclideanFitnessEpsilon());

//      setParams(registrationPtr);

      success = ICP2D(inCloudPCLPtr, targetPCLPtr, outCloudPCLPtr, registrationPtr);
//      success = ICP(inCloudPCLPtr, targetPCLPtr, outCloudPCLPtr, registrationPtr);


//          ROS_INFO_STREAM("ICP-CLUS--PERFORMING ICP ON CLUSTER ID " << clusterPtr->clusterID.data);

      if(success){
//            ROS_INFO_STREAM("ICP-CLUS-- SUCCESS ICP ON CLUSTER ID " << clusterPtr->clusterID.data);
        sensor_msgs::PointCloud2 ICPCombinedCloud;
        pcl::toROSMsg(*outCloudPCLPtr,ICPCombinedCloud);
        clusterPtr->icpCombinedCloud = ICPCombinedCloud;

        ROS_INFO_STREAM("ICP-CLUS-- ICP SCORE CLUSTER ID " << clusterPtr->clusterID.data << " = " << registrationPtr->getFitnessScore());
        clusterPtr->icp.score = registrationPtr->getFitnessScore();

        Eigen::Matrix4f transformation = registrationPtr->getFinalTransformation();
//        ROS_INFO_STREAM("ICP-CLUS-- GETTING getFinalTransformation ICP ON CLUSTER ID " << clusterPtr->clusterID.data << "\n" << registrationPtr->getFinalTransformation());

        transformation = transformation.inverse().eval();
//        ROS_INFO_STREAM("ICP-CLUS-- INVERTING getFinalTransformation ICP ON CLUSTER ID " << clusterPtr->clusterID.data << "\n" << transformation);

        // ROS_INFO_STREAM("ICP-CLUS-- ICP  TRANSFORMATION ON CLUSTER ID " << clusterPtr->clusterID.data << "\n" << transformation);
        clusterPtr->icp.transformStamped = Matrix4TFtoTransform(transformation);
        clusterPtr->icp.poseStamped = Matrix4TFtoPose(transformation);
        clusterPtr->icp.poseStamped.header = header_;
        ROS_INFO_STREAM("ICP-CLUS-- ICP  TRANSFORMATION ON CLUSTER ID " << clusterPtr->clusterID.data << "\n" << poseString(clusterPtr->icp.poseStamped.pose));

        //            ROS_INFO_STREAM("ICP-CLUS-- CLUSTER ID " << clusterPtr->clusterID.data << " POSE: " << clusterPtr->icp.poseStamped);
        //            ROS_INFO_STREAM("ICP-CLUS-- CLUSTER ID " << clusterPtr->clusterID.data << " TRANSFORM: " << clusterPtr->icp.transformStamped);
        //            std::cout << std::endl;
        //            ROS_INFO_STREAM("ICP-CLUS-- CLUSTER ID " << clusterPtr->clusterID.data << " ICP SCORE: " << clusterPtr->icp.score);
        //            ROS_INFO_STREAM("ICP-CLUS-- ICP THRESHOLD SCORE: " << ICP_Min_Score_);
        if(clusterPtr->icp.score < ICP_min_score_){
            clusterPtr->potentialDock.data = true;
            clusterPtr->icp.poseTextMarker = markPose(clusterPtr->icp.poseStamped.pose);
                ROS_WARN_STREAM("ICP-CLUS-- CLUSTER ID " << clusterPtr->clusterID.data << " POTENTIALLY IDENTIFIED AS DOCK");
        }
      } else {
        ROS_ERROR_STREAM("ICP-CLUS-- FAILED ICP ON CLUSTER ID " << clusterPtr->clusterID.data);
      }

  }
  ///////////////// END clusterICP /////////////////

  ///////////////// BEGIN clusterArrayICP /////////////////
  bool clusterArrayICP(docking::ClusterArray::Ptr clustersPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPCLPtr, docking::Cluster::Ptr dockClusterPtr)
  {
    //  ROS_INFO_STREAM("ICP-CLUS-ARRAY--BEGINNING ICP ON CLUSTERS ");
    dockClusterPtr->isDock.data = false;
    std::vector<docking::Cluster::Ptr> potential;
    std::vector<int> potentialIndex;
    long double lowestScore = ICP_min_score_;
    int lowestScoreIndex = -1;

    for(size_t i=0; i<clustersPtr->clusters.size();i++)
    {
      bool hasParallelLines = false;
      docking::Cluster::Ptr currentClusterPtr(new docking::Cluster());
      *currentClusterPtr = clustersPtr->clusters.at(i);

      // Consider only clusters that have enough lines
      // ROS_INFO_STREAM("ICP-CLUS-ARRAY--CURRENT CLUSTER " << i <<  " HAS " << currentClusterPtr->lines.lines.size() << " LINES");
      if(currentClusterPtr->lines.lines.size() < 2){
      //  ROS_INFO_STREAM("ICP-CLUS-ARRAY--SKIPPING ICP FOR CURRENT CLUSTER " << i <<  " WITH " << currentClusterPtr->lines.lines.size() << " LINES");
        continue;
      }

      // Eliminate Clusters With Parallel Lines (Lines found on circular Cluster)
      for(size_t j=0; j<currentClusterPtr->lines.lines.size(); j++){
        for(size_t k=j+1; k<currentClusterPtr->lines.lines.size(); k++){
          double angle = getAngle(currentClusterPtr->lines.lines.at(j),currentClusterPtr->lines.lines.at(k));
//          ROS_WARN_STREAM("ICP-CLUS-ARRAY--ANGLE BETWEEN LINES " << j << " AND " << k << "  = " << angle);
          if(angle < 20){
//            ROS_WARN_STREAM("ICP-CLUS-ARRAY--PARALLEL LINES FOUND IN CLUSTER " << i);
//            ROS_INFO_STREAM("SKIPPING ICP FOR CURRENT CLUSTER " << i <<  " WITH POSSIBLE PARALLEL LINES OF CIRCULAR OBJECT WITH ANGLE " << angle);
            hasParallelLines = true;
          }
        }
      }
      if(hasParallelLines){ continue;}


      clusterICP(currentClusterPtr, targetPCLPtr);
      clustersPtr->clusters.at(i) = *currentClusterPtr;
//      ROS_INFO_STREAM("CURRENT LOWEST SCORE VAL " << lowestScore);

      if(currentClusterPtr->potentialDock.data){
        ROS_WARN_STREAM("ICP-CLUS-ARRAY--DOCK POTENTIALLY IDENTIFIED AT CLUSTER " << i << " with score " << currentClusterPtr->icp.score);
        if(lowestScoreIndex == -1){ lowestScoreIndex = i;}
        potential.push_back(currentClusterPtr);
        potentialIndex.push_back(i);
//        ROS_INFO_STREAM("CLUSTER SCORE " << currentClusterPtr->icp.score);
//        ROS_INFO_STREAM("CURRENT LOWEST SCORE " << clustersPtr->clusters.at(lowestScoreIndex).icp.score);

        if(currentClusterPtr->icp.score <= clustersPtr->clusters.at(lowestScoreIndex).icp.score){
//          ROS_WARN_STREAM("SETTING LOWEST SCORE TO CURRENT CLUSTER SCORE ");
          lowestScoreIndex = i;
          lowestScore = clustersPtr->clusters.at(lowestScoreIndex).icp.score;
        }
//        ROS_INFO_STREAM("CURRENT LOWEST SCORE VAL " << lowestScore);
//        ROS_INFO_STREAM("CURRENT LOWEST SCORE INDEX " << lowestScoreIndex << " LOWEST SCORE " << clustersPtr->clusters.at(lowestScoreIndex).icp.score);
      }
    }

    if(lowestScoreIndex != -1){
      clustersPtr->clusters.at(lowestScoreIndex).isDock.data = true;
      *dockClusterPtr = clustersPtr->clusters.at(lowestScoreIndex);
    }

//      ROS_INFO_STREAM("ICP-CLUS-ARRAY--COMPLETED PERFORMING ICP ON CLUSTERS ");

  if(dockClusterPtr->isDock.data){
        ROS_WARN_STREAM("ICP-CLUS-ARRAY--DOCK SUCCESSFULLY IDENTIFIED AS CLUSTER " << lowestScoreIndex << " !!!!!!!!!!");
    return true;
  }
      ROS_ERROR_STREAM("ICP-CLUS-ARRAY--UNABLE TO IDENTIFY DOCK");
  return false;
  }
  ///////////////// END clusterArrayICP /////////////////

};

#endif // ICP_H
