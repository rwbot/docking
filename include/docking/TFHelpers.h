#ifndef TFHELPERS_H
#define TFHELPERS_H

#include <docking/Headers.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/impl/utils.h>
#include <tf2/impl/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/impl/utils.h>
#include <tf2/impl/convert.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

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

void printTF2Matrix(tf2::Matrix3x3 tf2m)
{
  tf2::Vector3 r0 = tf2m.getRow(0);
  tf2::Vector3 r1 = tf2m.getRow(1);
  tf2::Vector3 r2 = tf2m.getRow(2);
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", r0.x(), r0.y(), r0.z());
  printf("R = | %6.3f %6.3f %6.3f | \n", r1.x(), r1.y(), r1.z());
  printf("    | %6.3f %6.3f %6.3f | \n", r2.x(), r2.y(), r2.z());
//  printf("Translation vector :\n");
//  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void printTF2Quarternion(tf2::Quaternion tf2q)
{
  printf("QUARTERNION :\n");
  printf("    | %6.3f | \n", tf2q.x());
  printf("Q = | %6.3f | \n", tf2q.y());
  printf("    | %6.3f | \n", tf2q.z());
  printf("    | %6.3f | \n", tf2q.w());
}

void printTFQuarternion(tf::Quaternion tfq)
{
  printf("QUARTERNION :\n");
  printf("    | %6.3f | \n", tfq.x());
  printf("Q = | %6.3f | \n", tfq.y());
  printf("    | %6.3f | \n", tfq.z());
  printf("    | %6.3f | \n", tfq.w());
}

// Convert Pose->Orientation to String
std::string quaternionString(tf::Quaternion tfq){
  std::ostringstream qss;
  qss << std::fixed << std::setprecision(3) << "[ "<< tfq.getX() << ",  " << tfq.getY() << ",  " << tfq.getZ() << ",  " << tfq.getW() << " ]";
 return qss.str();
}

std::string quaternionString(tf2::Quaternion tf2q){
  std::ostringstream qss;
  qss << std::fixed << std::setprecision(3) << "[ "<< tf2q.getX() << ",  " << tf2q.getY() << ",  " << tf2q.getZ() << ",  " << tf2q.getW() << " ]";
 return qss.str();
}

// Convert Pose->Orientation to String
std::string quaternionString(geometry_msgs::Quaternion q){
  std::ostringstream qss;
  qss << std::fixed << std::setprecision(2) << "[ "<< q.x << ",  " << q.y << ",  " << q.z << ",  " << q.w << " ]";
 return qss.str();
}

std::string twistString(geometry_msgs::Twist t){
  std::ostringstream tss;
  tss << std::fixed << std::setprecision(3) << "[ X: "<< t.linear.x << ",  Z: " << t.angular.z <<  " ]";
 return tss.str();
}

std::string vectorString(tf2::Vector3 tf2v3){
  std::ostringstream qss;
  qss << std::fixed << std::setprecision(2) << "[ "<< tf2v3.getX() << ",  " << tf2v3.getY() << ",  " << tf2v3.getZ() << " ]";
 return qss.str();
}

std::string transformString(tf2::Transform tf2)
{
  // Convert Transform->Position to String
  tf2::Vector3 v = tf2.getOrigin();
  std::ostringstream positionSS;
  positionSS << std::fixed << std::setprecision(2) << "[ "<< v.getX() << ",  " << v.getY() << ",  " << v.getZ() << " ] ";

//  // Extract Yaw from Quarternion
//  std::ostringstream yawSS;
//  yawSS << std::fixed << std::setprecision(2) << "YAW: " << tf2::getYaw(tf2.getRotation()) << "\n";

  // Concatenate strings
  std::string transformString = positionSS.str() + quaternionString(tf2.getRotation());
//  std::string poseString = positionSS.str() + yawSS.str() + quarternionSS.str();
  return transformString;
}

std::string transformString(geometry_msgs::Transform tfMsg)
{
  tf2::Transform tf2;
  tf2::convert(tfMsg,tf2);
  return transformString(tf2);
}

std::string transformString(geometry_msgs::TransformStamped tfMsgStamped)
{
  std::ostringstream frameSS;
  frameSS << " frame_id: " << tfMsgStamped.header.frame_id;
  frameSS << " child_frame_id: " << tfMsgStamped.child_frame_id;

  tf2::Transform tf2;
  tf2::convert(tfMsgStamped.transform,tf2);

  // Concatenate strings
  std::string pose_string =  transformString(tf2) + frameSS.str() ;
  return pose_string;
  }


Eigen::Vector4f toEigen(pcl::ModelCoefficients pmc){
  pcl::ModelCoefficients::Ptr pmcPtr (new pcl::ModelCoefficients(pmc));
  pmcPtr->values.resize (4);
  pmcPtr->values[0] = 1.0;
  pmcPtr->values[1] = 2.0;
  pmcPtr->values[2] = 3.0;
  pmcPtr->values[3] = 4.0;
  Eigen::Vector4f ev4f (pmcPtr->values.data());
  return ev4f;
}

geometry_msgs::PoseStamped Matrix4TFtoPose(Eigen::Matrix4f m4f){

  Eigen::Matrix4d m4d = m4f.cast <double> ();
  geometry_msgs::PoseStamped poseStamped;

////  ROS_INFO_STREAM("Matrix4TFtoPose-- Creating tf::Transform object");
  tf2::Transform tf2;
  Eigen::Affine3d eigenAffine;
  eigenAffine.matrix() = m4d;
  geometry_msgs::TransformStamped tfMsg;
  tfMsg = tf2::eigenToTransform(eigenAffine);
  tf2::convert(tfMsg.transform,tf2);
  tf2::toMsg(tf2,poseStamped.pose);

  poseStamped.header.stamp = ros::Time::now();
  poseStamped.header.frame_id = "laser";

  return poseStamped;
}


geometry_msgs::TransformStamped Matrix4TFtoTransform(Eigen::Matrix4f m4f){

  Eigen::Matrix4d m4d = m4f.cast <double> ();
  Eigen::Affine3d eigenAffine;
  eigenAffine.matrix() = m4d;

  tf2::Transform tf2;
  geometry_msgs::TransformStamped tfMsg;
  tf2::convert(tf2,tfMsg.transform);

  tfMsg = tf2::eigenToTransform(eigenAffine);

//  ROS_INFO_STREAM("Matrix4TFtoTransformStamped-- Extracting Rotation Matrix in Eigen::Matrix3d ");
//  ROS_INFO_STREAM("Matrix4TFtoTransformStamped-- Copying Rotation Matrix into tf::Matrix3x3");

  tfMsg.header.stamp = ros::Time::now();
  tfMsg.header.frame_id = "laser";
  tfMsg.child_frame_id = "dock";


//  ROS_INFO_STREAM("Matrix4TFtoTransformStamped-- RETURNING TRANSFORM ");
//  ROS_INFO_STREAM("Matrix4TFtoTransformStamped-- TRANSFORM " << tfs.transform);

  return tfMsg;
}


// %Tag(poseString)%
std::string poseString(geometry_msgs::Pose pose, std::string label = std::string(), bool oneLine=true, bool sameLine=true) {
  // Convert Pose->Position to String
  std::ostringstream positionSS;
  positionSS << std::fixed << std::setprecision(2) << label ;
  if(!sameLine){ positionSS << std::endl; }
  positionSS << "[ "<< pose.position.x << ",  " << pose.position.y << ",  " << pose.position.z << " ] ";
  // positionSS << std::fixed << std::setprecision(2) << "[ "<< pose.position.x << ",  " << pose.position.y << " ] ";

  // Convert Pose->Orientation to String
  std::ostringstream quarternionSS;
  quarternionSS << std::fixed << std::setprecision(3) << "[ "<< pose.orientation.x << ",  " << pose.orientation.y << ",  " << pose.orientation.z << ",  " << pose.orientation.w << " ]";

  // Extract Yaw from Quarternion
  tf::Pose tfPose;
  tf::poseMsgToTF(pose, tfPose);
  double yaw = tf::getYaw(tfPose.getRotation());
  std::ostringstream yawSS;
  yawSS << std::fixed << std::setprecision(3) << " YAW: " << yaw;
  if(!oneLine)
    yawSS << "\n";

  // Concatenate strings
  std::string poseString = positionSS.str() + quarternionSS.str() + yawSS.str();
  return poseString;
}
// %EndTag(poseString)%

std::string poseString(geometry_msgs::PoseStamped pose, std::string label = std::string(), bool oneLine=true) {

  std::ostringstream frameSS;
  frameSS << " frame_id: " << pose.header.frame_id;

  // Concatenate strings
  std::string pose_string = poseString(pose.pose) + frameSS.str() ;
  return pose_string;
}

std::string yawString(double& yaw) {

  std::ostringstream yawSS;
  yawSS << std::fixed << std::setprecision(2) << " YAW: " << yaw;

  // Concatenate strings
  std::string yaw_string = yawSS.str();
  return yaw_string;
}

double getDistToGoal(double& x, double& y){
  double xx = x*x;
  double yy = y*y;
  double xxyy = xx + yy;
  double d = std::sqrt(xxyy);
  return d;}
void getDistToGoal(double& x, double& y, double& goalDist){  goalDist = getDistToGoal(x,y);}

double getDistBetweenPoses(geometry_msgs::Pose& cp, geometry_msgs::Pose& tp){
//    ROS_INFO_STREAM("GETTING DISTANCE BETWEEN POSES" << poseString(cp) << poseString(tp));
  double delta_x = tp.position.x - cp.position.x;
  double delta_y = tp.position.y - cp.position.y;
//    ROS_INFO_STREAM("DELTA X " << delta_x << "DELTA X^2 " << (delta_x*delta_x));
//    ROS_INFO_STREAM("DELTA Y " << delta_y << "DELTA Y^2 " << (delta_y*delta_y));
  double d = std::sqrt(delta_x*delta_x + delta_y*delta_y);
//    ROS_INFO_STREAM("DISTANCE BETWEEN POSES " << d);
  return d;
}

double getYaw(geometry_msgs::Quaternion& qMsg){
  ROS_INFO_STREAM("Getting Yaw from Quaternion" << quaternionString(qMsg));
  tf::Quaternion qTF;
  tf::quaternionMsgToTF(qMsg, qTF);
  ROS_INFO_STREAM("Quaternion TF" << quaternionString(qTF)); //printTFQuarternion(qTF);
  qTF.normalize();
  ROS_INFO_STREAM("Quaternion TF Normalized" << quaternionString(qTF)); //printTFQuarternion(qTF);
//    double yaw = angles::normalize_angle(tf::getYaw(qTF));
  double yaw = tf::getYaw(qTF);
  ROS_INFO_STREAM("Finished Getting Yaw from Quaternion YAW: " << yaw);
  return yaw;
}

//  double getYaw(geometry_msgs::Pose& poseMsg){
//    ROS_INFO_STREAM("Getting Yaw from Pose" << poseString(poseMsg));
//    tf::Pose poseTF;
//    tf::poseMsgToTF(poseMsg, poseTF);
//    double yaw = tf::getYaw(poseTF.getRotation());
//    ROS_INFO_STREAM("Finished Getting Yaw from Pose YAW: " << yaw);
//    tf2::Quaternion qTF;
//    tf2::convert(poseMsg.orientation , qTF);
//    yaw = tf2::getYaw(qTF);
//    ROS_INFO_STREAM("Finished Getting Yaw from Quaternion YAW: " << yaw);
//    return yaw;
//  }

geometry_msgs::Quaternion getQuaternion(double& yaw){
  ROS_INFO_STREAM("Getting Quaternion from Yaw: " << yaw);
  tf::Quaternion qTF = tf::createQuaternionFromYaw(yaw);
  ROS_INFO_STREAM("Quaternion TF" << quaternionString(qTF)); //printTFQuarternion(qTF);
  qTF.normalize();
  ROS_INFO_STREAM("Quaternion TF Normalized" << quaternionString(qTF)); //printTFQuarternion(qTF);
  geometry_msgs::Quaternion qMsg;
  tf::quaternionTFToMsg(qTF, qMsg);
  ROS_INFO_STREAM("Finished Getting Quaternion from Yaw" << quaternionString(qMsg));
  return qMsg;
}


void syncTFData(tf2::Transform& tf2, geometry_msgs::TransformStamped& tfMsg, geometry_msgs::PoseStamped& poseStamped, std::string frameID, std::string childFrameID)
{
//    ROS_INFO_STREAM("SYNCING TF --> TF MSF and POSE" << transformString(tf2));
  // tf2 to tfMSG
  tf2::convert(tf2,tfMsg.transform);
  tfMsg.header.frame_id = frameID;
  tfMsg.child_frame_id = childFrameID;
  tfMsg.header.stamp = ros::Time::now();
//    ROS_INFO_STREAM("SYNCING TF MSG FROM TF " << transformString(tfMsg));
  // tf2 to Pose
  tf2::toMsg(tf2,poseStamped.pose);
  poseStamped.header.frame_id = frameID;
  poseStamped.header.stamp = tfMsg.header.stamp;
//    ROS_INFO_STREAM("SYNCING POSE FROM TF " << poseString(poseStamped));
}

void syncTFData(geometry_msgs::TransformStamped& tfMsg, tf2::Transform& tf2, geometry_msgs::PoseStamped& poseStamped)
{
//    ROS_INFO_STREAM("SYNCING TF MSG --> TF and POSE" << transformString(tfMsg));
  // tfMSG to tf2
  tf2::convert(tfMsg.transform,tf2);
//    ROS_INFO_STREAM("SYNCING TF FROM TF MSG" << transformString(tf2));
  // tf2 to Pose
  tf2::toMsg(tf2,poseStamped.pose);
  poseStamped.header.stamp = tfMsg.header.stamp;
  poseStamped.header.frame_id = tfMsg.header.frame_id;
//    ROS_INFO_STREAM("SYNCING POSE FROM TF MSG" << poseString(poseStamped));
}

void syncTFData(geometry_msgs::PoseStamped& poseStamped, tf2::Transform& tf2, geometry_msgs::TransformStamped& tfMsg, std::string childFrameID)
{
//    ROS_INFO_STREAM("SYNCING POSE --> TF and TF MSG " << poseString(poseStamped));
  // pose to tf2
  tf2::fromMsg(poseStamped.pose,tf2);
//    ROS_INFO_STREAM("SYNCING TF FROM POSE " << transformString(tf2));
  // tf2 to TFMsg
  tf2::convert(tf2,tfMsg.transform);
  tfMsg.header.frame_id = poseStamped.header.frame_id;
  tfMsg.child_frame_id = childFrameID;
  tfMsg.header.stamp = poseStamped.header.stamp;
//    ROS_INFO_STREAM("SYNCING TF MSG FROM POSE " << transformString(tfMsg));
}

#endif // TFHELPERS_H
