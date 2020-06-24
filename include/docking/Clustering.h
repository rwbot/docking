#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <docking/Headers.h>

class Clustering {

public:
  Clustering(){

  }
  ~Clustering(){}

  std_msgs::Header header_;

  void setHeader(std_msgs::Header header){
    header_ = header;
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

    validateDimensions(cluster.bbox.dimensions.x, cluster.bbox.dimensions.y, cluster.bbox.dimensions.z);
    marker.scale.x = cluster.bbox.dimensions.x;
    marker.scale.y = cluster.bbox.dimensions.y;
    marker.scale.z = cluster.bbox.dimensions.z;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.15f;

    marker.lifetime = ros::Duration();
    //       marker.lifetime = ros::Duration(0.5);
    return marker;
  }



///////////////// BEGIN ROSIFY CLUSTER /////////////////
docking::Cluster rosifyCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudPCLPtr,
                               pcl::PointIndices::Ptr &indicesPCLPtr) {

  docking::Cluster clusterMsg;
  sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);
  pcl_msgs::PointIndicesPtr indicesMsgPtr(new pcl_msgs::PointIndices);

  // ROS_INFO_STREAM(CONVERTING CURRENT CLUSTER TO ROS MSG");
  pcl::toROSMsg(*cloudPCLPtr, *cloudMsgPtr);
  clusterMsg.cloud = *cloudMsgPtr;
  // ROS_INFO_STREAM(CONVERTING INDICES TO ROS MSG");
  clusterMsg.points.indices = indicesPCLPtr->indices;

  clusterMsg.bbox = getBoundingBoxClusterOriented(cloudPCLPtr);
  clusterMsg.bbox.marker = markCluster(clusterMsg);

  return clusterMsg;
}
///////////////// END ROSIFY CLUSTER /////////////////

///////////////// BEGIN ROSIFY CLUSTER /////////////////
docking::Cluster rosifyCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudPCLPtr) {
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
//  void publishMarker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudPCLPtr,
//                     pcl::PointIndices::Ptr &indicesPCLPtr,
//                     pcl::ModelCoefficients::Ptr &coefficientsPCLPtr) {
//    sensor_msgs::PointCloud2Ptr cloudMsgPtr(new sensor_msgs::PointCloud2);
//  }
///////////////// END PUBLISH MARKER /////////////////

///////////////// BEGIN BOUNDING BOX CLUSTER /////////////////
/// \brief getBoundingBoxCluster
/// \param cluster
/// \return bbox
///
docking::BoundingBox getBoundingBoxCluster(docking::Cluster cluster) {
  docking::BoundingBox bbox;

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(cluster.cloud, cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));

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

  validateDimensions(bbox.dimensions.x, bbox.dimensions.y, bbox.dimensions.z);

  bbox.area = (bbox.max.x - bbox.min.x) * (bbox.max.y - bbox.min.y);
  bbox.volume = (bbox.area) * (bbox.max.z - bbox.min.z);

  bbox.header = header_;

  return bbox;
}
///////////////// END BOUNDING BOX CLUSTER /////////////////

///////////////// BEGIN BOUNDING BOX CLUSTER ORIENTED /////////////////
/// \brief getBoundingBoxClusterOriented
/// \param cluster
/// \return bbox
///
docking::BoundingBox getBoundingBoxClusterOriented(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr origCloudPtr) {
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjectedPtr(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud(*origCloudPtr, *cloudPointsProjectedPtr,
                           projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjectedXYZPtr(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloudPointsProjectedPtr, *cloudPointsProjectedXYZPtr);

//    printDebugCloud(cloudPointsProjectedXYZPtr);

  //        pcl::PointXYZ minPointProjected, maxPointProjected;
  pcl::PointXYZRGB minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjectedPtr, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal =
      0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  // Final transform
  const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
  const Eigen::Vector3f bboxPosition = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

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

  //        @TODO: Points still in dock frame
  //              Transform min and max points back into laser frame
  //        bboxMsg.dimensions.x = bboxMsg.max.x - bboxMsg.min.x ;
  bboxMsg.dimensions.x = 0.0777; //
  bboxMsg.dimensions.y = bboxMsg.max.y - bboxMsg.min.y;
  bboxMsg.dimensions.z = bboxMsg.max.z - bboxMsg.min.z;
  validateDimensions(bboxMsg.dimensions.x,bboxMsg.dimensions.y,bboxMsg.dimensions.z);

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

///////////////// BEGIN CLUSTER POINTS /////////////////
/// \brief ClusterPoints
/// \param cloud
/// \param clusterTolerance
/// \param minSize
/// \param maxSize
/// \return
///
void ClusterPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, docking::ClusterArray::Ptr clustersPtr, double clusterTolerance, int minSize, int maxSize) {
  //std::cout << std::endl;
  // ROS_INFO_STREAM();
//    ROS_INFO_STREAM("CLUSTERING Called: ");
//    ROS_INFO_STREAM("CLUSTERING: clustersPtr.header " << clustersPtr->header);
  //        ROS_INFO_STREAM("");
//    ROS_INFO_STREAM("Cluster tolerance: " << clusterTolerance << " Min Points: " << minSize << " Max Points: " << maxSize);

  docking::ClusterArray clusters;

  pcl::PointCloud<pcl::PointXYZRGB> combinedClustersCloud;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(inCloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(inCloud);
  ec.extract(cluster_indices);


//    ROS_INFO_STREAM("CLUSTERING: " << cluster_indices.size()<< " clusters found");
  if(cluster_indices.size() == 0){
//    ROS_WARN_STREAM("CLUSTERING: NO CLUSTERS FOUND ");
    return;
  }


  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clustersPCLVector;
  int i = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++i) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudClusterPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
      cloudClusterPtr->points.push_back(inCloud->points[*pit]);
    }

    cloudClusterPtr->width = cloudClusterPtr->points.size();
    cloudClusterPtr->height = 1;
    cloudClusterPtr->is_dense = true;

//      ROS_INFO_STREAM("CLUSTERING: Cluster " << i << " has "<< cloudClusterPtr->points.size() << " points");
    // Add current cluster to list of clusters
    // ROS_INFO_STREAM("COLORING CLUSTER");
    ColorCloud(cloudClusterPtr, i);
    // ROS_INFO_STREAM("COMBINING CLUSTER");
    combinedClustersCloud += *cloudClusterPtr;
    // ROS_INFO_STREAM("ADDING CLUSTER TO VECTOR");
    clustersPCLVector.push_back(cloudClusterPtr);
    pcl::PointIndicesPtr currentPointIndicesPtr(new pcl::PointIndices(*it));
    // ROS_INFO_STREAM("ROSIFYING CLUSTER");
    docking::Cluster clusterMsg = rosifyCluster(cloudClusterPtr, currentPointIndicesPtr);
    clusterMsg.clusterID.data = i;
    clusterMsg.header = clusterMsg.cloud.header = clusterMsg.points.header = clustersPtr->header;

//      clusterMsg.bbox = getBoundingBoxClusterOriented(cloudClusterPtr);
//      clusterMsg.bbox.marker = markCluster(clusterMsg);

    clustersPtr->clusters.push_back(clusterMsg);
  }
//     ROS_INFO_STREAM("CLUSTERING: CONVERTING COMBINED CLUSTER TO ROS MSG");
  pcl::toROSMsg(combinedClustersCloud, clustersPtr->combinedCloud);
  clustersPtr->combinedCloud.header = clustersPtr->header;
//    ROS_INFO_STREAM("CLUSTERING: FOUND " << clustersPtr->clusters.size() << " CLUSTERS");
}
///////////////// END CLUSTER POINTS /////////////////
};

#endif // CLUSTERING_H
