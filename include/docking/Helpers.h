#ifndef HELPERS_H
#define HELPERS_H

#include <docking/Headers.h>
// Auto-generated from cfg/ directory.
#include <docking/BoundingBox.h>
#include <docking/Cluster.h>
#include <docking/ClusterArray.h>
#include <docking/Dock.h>
#include <docking/Line.h>
#include <docking/LineArray.h>
#include <docking/MinMaxPoint.h>
// #include <docking/DetectionNodeConfig.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

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


void printLineInfo(docking::Line line) {
  ROS_INFO_STREAM("LINE ID "<< line.lineID.data << " has "
                  << line.points.indices.size() << " points and "
                  << " and Centroid X:" << line.centroid.position.x
                  << " Y:" << line.centroid.position.y
                  << " Z:" << line.centroid.position.z << std::endl
                  << "Model Coefficients AX:" << line.coefficients.values[0]
                  << " BY:" << line.coefficients.values[1]
                  << " CZ:" << line.coefficients.values[2]
                  << " D:" << line.coefficients.values[3]);
}

std::uint32_t getRGBUI32(int ri, int gi, int bi) {
  //      std::uint8_t r(ri), g(gi), b(bi);
  //      std::uint32_t rgb32 = ((std::uint32_t)r << 16 | (std::uint32_t)g <<
  //      8 | (std::uint32_t)b);
  // ----------------------------------------------------------------------------------------
  //      std::uint32_t rgb32 = ( static_cast<std::uint32_t>(r) << 16 |
  //                            static_cast<std::uint32_t>(g) << 8 |
  //                            static_cast<std::uint32_t>(b)
  //                          );
  // ----------------------------------------------------------------------------------------
  std::uint32_t rgb32 =
      ((std::uint32_t)ri << 16 | (std::uint32_t)gi << 8 | (std::uint32_t)bi);
  //      int rgb = ((int)ri) << 16 | ((int)gi) << 8 | ((int)bi);
  //      std::cout << "CONVERT RGB " << rgb << " R " << unsigned(r) << " G "
  //      << unsigned(g) << " B " << unsigned(b) <<std::endl; std::cout <<
  //      "CONVERT RGB " << rgb32 << " Ri " << ri << " Gi " << gi << " Bi " <<
  //      bi <<std::endl;
  return rgb32;
}

///////////////// BEGIN COLOR CLOUD/////////////////
/// \brief ColorCloud
/// \param inCloudPtr
/// \return
///
void ColorCloud(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cPtr,
                int color) {
  color = color % 4;
  for (size_t i = 0; i < cPtr->points.size(); ++i) {
    int rgb = cPtr->points[i].rgb;

    //          std::cout << "BEF RGB " << cPtr->points[i].rgb << " R " <<
    //          cPtr->points[i].r << " G " << cPtr->points[i].g << " B " <<
    //          cPtr->points[i].b << " val " << val <<std::endl;
    //          ROS_INFO_STREAM("BEF RGB " << int(cPtr->points[i].rgb) << " R
    //          " << unsigned(cPtr->points[i].r )<< " G " <<
    //          +(cPtr->points[i].g) << " B " << +cPtr->points[i].b);
    //          ROS_INFO_STREAM("CUR cit " << " R " << r << " G " << g << " B-
    //          " << b); ROS_INFO_STREAM("CUR cit " << " R- " <<
    //          static_cast<unsigned char>(cit->r) << " G- " <<
    //          static_cast<unsigned char>(cit->g) << " B- " << (cit->b));
    //          std::cout << "CUR cit "<< " R: " << static_cast<unsigned
    //          char>(cit->r) << " G: " << cit->g << " B: " << cit->b <<
    //          std::endl;

    std::uint32_t rgb32;
    switch (color) {
    case 0:
      //              ROS_INFO_STREAM("COLORING RED");
      rgb32 = getRGBUI32(255, 0, 0);
      break;
    case 1:
      //              ROS_INFO_STREAM("COLORING GREEN");
      rgb32 = (getRGBUI32(0, 255, 0));
      break;
    case 2:
      //              ROS_INFO_STREAM("COLORING BLUE");
      rgb32 = (getRGBUI32(0, 0, 255));
      break;
    case 3:
      //              ROS_INFO_STREAM("COLORING PINK");
      rgb32 = (getRGBUI32(255, 20, 147));
      break;
    }
    cPtr->points[i].rgb = *reinterpret_cast<float *>(&rgb32);
  }
}
///////////////// END COLOR CLOUD /////////////////

void extractRGBUI32(int rgb) {
  std::uint8_t r = (rgb >> 16) & 0x0000ff;
  std::uint8_t g = (rgb >> 8) & 0x0000ff;
  std::uint8_t b = (rgb)&0x0000ff;

  //      std::cout << "EXTRACT RGB " << +rgb << " R " << int(r) << " G " <<
  //      unsigned(g) << " B " << +b << std::endl;
}

bool validateDimensions(float& x, float& y, float& z){
  bool allValid = true;
  if (x < 0.01f) {
    x = 0.169;
    allValid = false;
  }
  if (y < 0.01f) {
    y = 0.169;
    allValid = false;
  }
  if (z < 0.01f) {
    z = 0.169;
    allValid = false;
  }
  return allValid;
}

bool validateDimensions(double& x, double& y, double& z){
  bool allValid = true;
  if (x < 0.01) {
    x = 0.169;
    allValid = false;
  }
  if (y < 0.01) {
    y = 0.169;
    allValid = false;
  }
  if (z < 0.01) {
    z = 0.169;
    allValid = false;
  }
  return allValid;
}




///////////////// BEGIN MARK CLUSTER /////////////////
//    visualization_msgs::Marker
//    mark_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster,
//    std::string ns ,int id, float r, float g, float b)
//visualization_msgs::Marker markPose(docking::Cluster cluster)
visualization_msgs::Marker markPose(geometry_msgs::Pose pose)
{
  uint32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.ns = "docking";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.text = poseString(pose,"ICP POSE");

  marker.pose.position.y = 0.8;
  marker.pose.position.x = 0.17;

//  marker.scale.x = 0.5;
//  marker.scale.y = 0.5;
  marker.scale.z = 0.1;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.5f;
  marker.color.a = 1.0f;

  marker.lifetime = ros::Duration();
  //       marker.lifetime = ros::Duration(0.5);
  return marker;
}


#endif // HELPERS_H
