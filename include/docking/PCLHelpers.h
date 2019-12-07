#ifndef PCLHELPERS_H
#define PCLHELPERS_H

// Auto-generated from cfg/ directory.
#include <docking/Helpers.h>
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
#include <geometry_msgs/Point.h>
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

namespace docking{


geometry_msgs::Point pointPCLToMSG(pcl::PointXYZ point) {
  geometry_msgs::Point pointMsg;
  // Get Min Point
  pointMsg.x = double(point.x);
  pointMsg.y = double(point.y);
  pointMsg.z = double(point.z);
  return pointMsg;
}

geometry_msgs::Point pointPCLToMSG(pcl::PointXYZRGB point) {
  geometry_msgs::Point pointMsg;
  // Get Min Point
  pointMsg.x = double(point.x);
  pointMsg.y = double(point.y);
  pointMsg.z = double(point.z);
  return pointMsg;
}

docking::MinMaxPoint getMinMaxPointMsg(typename pcl::PointCloud<pcl::PointXYZ>::Ptr inCloudPtr) {
  pcl::PointXYZ minPointPCL, maxPointPCL;
  pcl::getMinMax3D(*inCloudPtr, minPointPCL, maxPointPCL);
  docking::MinMaxPoint minMaxMsg;
  minMaxMsg.min = pointPCLToMSG(minPointPCL);
  minMaxMsg.max = pointPCLToMSG(maxPointPCL);
//  minMaxMsg.header = header_;
  return minMaxMsg;
}

docking::MinMaxPoint getMinMaxPointMsg(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPtr) {
  pcl::PointXYZRGB minPointPCL, maxPointPCL;
  pcl::getMinMax3D(*inCloudPtr, minPointPCL, maxPointPCL);
  docking::MinMaxPoint minMaxMsg;
  minMaxMsg.min = pointPCLToMSG(minPointPCL);
  minMaxMsg.max = pointPCLToMSG(maxPointPCL);
//  minMaxMsg.header = header_;
  return minMaxMsg;
}

jsk_recognition_msgs::Segment minMaxToSegment(docking::MinMaxPoint minMax) {
  jsk_recognition_msgs::Segment segment;
  segment.start_point = minMax.min;
  segment.end_point = minMax.max;
  return segment;
}

float comparePoints(geometry_msgs::Point p1, geometry_msgs::Point p2){
  float avgDelta;
  avgDelta += abs(p2.x - p1.x);
  avgDelta += abs(p2.y - p1.y);
  avgDelta += abs(p2.z - p1.z);
  avgDelta = avgDelta/3;
  if(avgDelta < 0){
    avgDelta = 0;
  }
  return avgDelta;
}

float compareSegments(jsk_recognition_msgs::Segment s1, jsk_recognition_msgs::Segment s2){
  float avgDelta;
  avgDelta += comparePoints(s1.start_point, s2.start_point);
  avgDelta += comparePoints(s1.end_point, s2.end_point);
  avgDelta = avgDelta/2;
  if(avgDelta < 0){
    avgDelta = 0;
  }
  return avgDelta;
}

float compareOrientation(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2){
  float avgDelta;
  avgDelta += abs(q2.x - q1.x);
  avgDelta += abs(q2.y - q1.y);
  avgDelta += abs(q2.z - q1.z);
  avgDelta += abs(q2.w - q1.w);
  avgDelta = avgDelta/4;
  if(avgDelta < 0){
    avgDelta = 0;
  }
  return avgDelta;
}

float comparePoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
  float positionDelta = comparePoints(p1.position, p2.position);
  float orientationDelta = compareOrientation(p1.orientation, p2.orientation);
  float avgDelta = positionDelta + orientationDelta;
  avgDelta = avgDelta/2;
  if(avgDelta < 0){
    avgDelta = 0;
  }
  return avgDelta;
}

float compareCoefficients(pcl_msgs::ModelCoefficients c1, pcl_msgs::ModelCoefficients c2){
  float avgDelta;
  avgDelta = abs(c2.values[0] - c1.values[0]);
  avgDelta = abs(c2.values[1] - c1.values[1]);
  avgDelta = abs(c2.values[2] - c1.values[2]);
  avgDelta = abs(c2.values[3] - c1.values[3]);
  avgDelta = avgDelta/4;
  if(avgDelta < 0){
    avgDelta = 0;
  }
  return avgDelta;
}

int comparePointIndices(pcl_msgs::PointIndices pi1, pcl_msgs::PointIndices pi2){
  pcl_msgs::PointIndices diffPoints;
  int numDiffPoints;
//  int size1 = pi1.indices.size();
//  int size2 = pi2.indices.size();
  int biggerSize; int smallerSize;
  pcl_msgs::PointIndices bigger;
  pcl_msgs::PointIndices smaller;

  if(pi2.indices.size() == pi1.indices.size()){
    return 0.0;
  } else if (pi2.indices.size() > pi1.indices.size()){
    biggerSize = pi2.indices.size();
    bigger = pi2;
    smaller = pi1;
    smallerSize = pi1.indices.size();
  } else if (pi2.indices.size() < pi1.indices.size()){
    biggerSize = pi1.indices.size();
    bigger = pi1;
    smaller = pi2;
    smallerSize = pi2.indices.size();
  }

  for(int i=0; i<smallerSize; i++){
    for(int j=0; j<biggerSize; j++){
      if(smaller.indices.at(i) == bigger.indices.at(j)){
        bigger.indices.erase(bigger.indices.begin()+j);
        biggerSize -= 1;
      }
    }
  }

    numDiffPoints = bigger.indices.size();
    return numDiffPoints;
  }



}

#endif // PCLHELPERS_H
