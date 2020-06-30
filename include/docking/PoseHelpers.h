#ifndef POSEHELPERS_H
#define POSEHELPERS_H

#include <docking/Headers.h>
// Auto-generated from cfg/ directory.
#include <docking/BoundingBox.h>
#include <docking/Cluster.h>
#include <docking/ClusterArray.h>
#include <docking/Dock.h>
#include <docking/Line.h>
#include <docking/LineArray.h>
#include <docking/MinMaxPoint.h>
//#include <docking/SegmentLineConfig.h>




  void stepPose(geometry_msgs::PoseStamped& poseOld, geometry_msgs::Twist& t, double& time_step){
    ROS_INFO_STREAM("Started Stepping Pose with Twist " << twistString(t));
    geometry_msgs::PoseStamped poseDelta, poseNew;
    ros::Duration time_step_duration(time_step);
    poseOld.header.stamp = poseOld.header.stamp + time_step_duration;

    double yawOld = getYaw(poseOld.pose.orientation), yawNew=0;
    tf2::Quaternion qOld, qDelta, qNew;
    tf2::convert(poseOld.pose.orientation , qOld);


    poseDelta.pose.position.x = (t.linear.x * cos(yawOld)  -  t.linear.y * sin(yawOld)) * time_step;
    poseDelta.pose.position.y = (t.linear.x * sin(yawOld)  +  t.linear.y * cos(yawOld)) * time_step;
    double yawDelta = t.angular.z * time_step;
    poseDelta.pose.orientation = getQuaternion(yawDelta);


    poseNew.pose.position.x = poseOld.pose.position.x + poseDelta.pose.position.x;
    poseNew.pose.position.y = poseOld.pose.position.y + poseDelta.pose.position.y;
    yawNew = yawOld + yawDelta;
    poseNew.pose.orientation = getQuaternion(yawNew);
    qDelta.setRPY(0,0,yawDelta);
    qNew = qDelta * qOld;
    qNew.normalize();

    ROS_INFO_STREAM("ORIG  Pose           " << poseString(poseOld.pose) << " YAW: " << yawOld);

    ROS_INFO_STREAM("DELTA Pose (YAW calc)" << poseString(poseDelta.pose)  << " YAW: " << yawDelta);
    tf2::convert(qDelta, poseDelta.pose.orientation);
    ROS_INFO_STREAM("DELTA Pose (Qua calc)" << poseString(poseDelta.pose)  << " YAW: " << yawDelta);

    ROS_INFO_STREAM("STEPD Pose (YAW calc)" << poseString(poseNew.pose)  << " YAW: " << yawNew);
    tf2::convert(qNew, poseNew.pose.orientation);
    ROS_INFO_STREAM("STEPD Pose (Qua calc)" << poseString(poseNew.pose)  << " YAW: " << yawNew);

    poseOld.pose = poseNew.pose;
    ROS_INFO_STREAM("Finished Stepping Pose");
  }


  tf2::Transform getDeltaTF(tf2::Transform curTF, geometry_msgs::Twist& t, double& time_step){
    tf2::Transform deltaTF;
    ROS_INFO_STREAM("Started Stepping Pose with Twist " << twistString(t));

    tf2::Quaternion qOld, qDelta, qNew;
    tf2::convert(curTF.getRotation(), qOld);
    double yawOld = tf2::getYaw(qOld);
    double deltaX, deltaY, deltaYaw, rho;
    rho = t.linear.x * time_step;
    deltaYaw = t.angular.z * time_step;

//    deltaX = (t.linear.x * cos(deltaYaw)  -  t.linear.y * sin(deltaYaw)) * time_step;
//    deltaY = (t.linear.x * sin(deltaYaw)  +  t.linear.y * cos(deltaYaw)) * time_step;
    deltaX = rho * cos(deltaYaw);
    deltaY = rho * sin(deltaYaw);

    tf2::Vector3 deltaOrigin(deltaX, deltaY, curTF.getOrigin().getZ());
    deltaTF.setOrigin(deltaOrigin);
    qDelta.setRPY(0,0,deltaYaw);
    deltaTF.setRotation(qDelta);

    std::ostringstream positionSS;
    positionSS << std::fixed << std::setprecision(2) << "DELTA X POSITION " << deltaX << " DELTA Y POSITION " << deltaY  << " ";

    ROS_INFO_STREAM(positionSS.str());

    ROS_INFO_STREAM("ORIG  TF " << transformString(curTF) <<  yawString(yawOld));
    ROS_INFO_STREAM("DELTA TF " << transformString(deltaTF)  << yawString(deltaYaw));

    return deltaTF;
//    tf2::convert(qDelta, poseDelta.pose.orientation);
//    ROS_INFO_STREAM("DELTA TF (Qua calc)" << poseString(poseDelta.pose)  << " YAW: " << yawDelta);
  }


  tf2::Transform getProjectionToTargetTF(tf2::Transform& base2Proj, tf2::Transform& base2Target){
    // AtoC = AtoB * BtoC;
    tf2::Transform proj2Base, proj2Target;
    proj2Base = base2Proj.inverse();
    proj2Target = proj2Base * base2Target;
    ROS_INFO_STREAM("GETTING TF FROM PROJECTION TO TARGET " << transformString(proj2Target));
    return proj2Target;
  }



#endif // POSEHELPERS_H
