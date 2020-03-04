#ifndef POSECONTROLLERNODE_H
#define POSECONTROLLERNODE_H

//#include <docking/Headers.h>
#include <docking/Helpers.h>
#include <docking/PCLHelpers.h>
#include <ros/ros.h>
#include <docking/PoseControllerConfig.h>
#include <docking/Plan.h>
// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/impl/utils.h>
#include <tf2/impl/convert.h>
#include <tf2/transform_datatypes.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>

class PoseControllerNode
{
public:
  PoseControllerNode(ros::NodeHandle& nh) :
    nh_(nh), tfListener_(tfBuffer_)
  {
//    nh_ = nh;
    ROS_INFO_STREAM("INITIALIZING POSE CONTROLLER NODE OBJECT");

    ROS_INFO_STREAM("STARTING DYNAMIC RECONFIGURE SERVER");
    startDynamicReconfigureServer();
//    startPoseSub(dock_frame_);
    ROS_INFO_STREAM("PoseControllerNode: Subscribing to 'dock_truth'");
    startPub();
    startPoseSub("dock_pose_gazebo");
    ros::Duration time_step_duration(time_step_);
    time_step_duration_ = time_step_duration;

    plan_.header.frame_id = plan_.path.header.frame_id = plan_.poseArray.header.frame_id = "base_link";
  }
  ~PoseControllerNode(){}

  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;  // Publisher of commands
  ros::Publisher path_pub_;  // Publisher of paths
  ros::Publisher pose_array_pub_;  // Publisher of pose array
  ros::Subscriber dockPoseSub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
//  tf::TransformListener tfListener_;
  docking::Plan plan_;

  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<docking::PoseControllerConfig> dr_srv_;
  //! Dynamic Reconfig Variables
  std::string dock_frame_;
  double omega_max_;
  double k1_;  // ratio in change of theta to rate of change in r
  double k2_;  // speed at which we converge to slow system
  double curvature_max_;
  double beta_;  // how fast velocity drops as k increases
  double lambda_;  // ??
  double lin_vel_min_;
  double lin_vel_max_;
  int path_steps_;
  double time_step_;
  ros::Duration time_step_duration_;

  //! Control System Variables
  double goalDist_;
  double deltaAngle_;
  double phi_;
  double deltaControl_;
  double curvature_;
  double v_;
  double omega_;
  double omegaBounded_;


  // Declaration and Definition
  void startDynamicReconfigureServer() {
  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server values can be overwritten.
    dynamic_reconfigure::Server<docking::PoseControllerConfig>::CallbackType cb;
    cb = boost::bind(&PoseControllerNode::configCallback, this, _1, _2);
    dr_srv_.setCallback(cb);
  }

  //! Callback function for dynamic reconfigure server.
  void configCallback(docking::PoseControllerConfig &config, uint32_t level __attribute__((unused))) {
    omega_max_ = config.omega_max;
    k1_ = config.k1;
    k2_ = config.k2;
    curvature_max_ = config.curvature_max;
    beta_ = config.beta;
    lambda_ = config.lambda;
    lin_vel_min_ = config.v;
    lin_vel_max_ = config.v;
    path_steps_ = config.path_steps;
    time_step_ = config.time_step;
  }

  void startPub() {
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("docking/cmd_vel", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("docking/path", 10);
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("docking/pose_array", 10);
  }

//  void startPoseSub(std::string& dockPoseTopic)
  void startPoseSub(std::string dockPoseTopic)
  {
    dockPoseSub_ = nh_.subscribe(dockPoseTopic, 1, &PoseControllerNode::dockPoseCallback, this);
  }

  void clearGlobals(){
    plan_.path.poses.clear();
    plan_.poseArray.poses.clear();
    plan_.twists.clear();
  }

  ///////////////// BEGIN CALCULATE PLAN /////////////////
  /// \brief calculateCurrentApproach
  /// \param targetPose
  /// \return
  ///

  bool calculatePlan(geometry_msgs::PoseStamped& targetPose){
    std::cout << std::endl;
    clearGlobals();
    geometry_msgs::Twist twist;
    int steps = 0;

    ROS_INFO_STREAM("**************** BEGINNING CALCULATING PLAN ****************");
    tf2::Transform base2ProjectionTF, robotTF, base2TargetTF, proj2TargetTF;
    geometry_msgs::TransformStamped base2ProjectionTFMsg, robotTFMsg, base2TargetTFMsg, proj2TargetTFMsg;
    geometry_msgs::PoseStamped base2ProjectionPose, robotPose, base2TargetPose, proj2TargetPose;
    std::string projectionFrameID="projection", robotFrameID="base_link", targetFrameID="target";

    // Transform targetPose into base_link frame
    try {
      ROS_INFO_STREAM("ORIGINAL TARGET POSE MSG" << poseString(targetPose.pose));
      ROS_INFO_STREAM(targetPose);

//      syncTFData(targetPose,base2TargetTF,base2TargetTFMsg,targetFrameID);

//      tf2::fromMsg(targetPose.pose,targetTF);
//      ROS_INFO_STREAM("INITIALIZING TARGET TRANSFORM FROM POSE" << transformString(targetTF));
//      tf2::convert(targetTF,targetTFMsg.transform);
//      targetTFMsg.header.frame_id = "base_link";
//      targetTFMsg.child_frame_id = "projection";
//      ROS_INFO_STREAM("INITIALIZING TARGET TF MSG FROM TF " << targetTFMsg);
      ROS_INFO_STREAM("TRANSFORMING TARGET POSE FROM " << targetPose.header.frame_id << " TO " << robotFrameID);
//      tfBuffer_.transform(base2TargetTFMsg,base2TargetTFMsg,"base_link");
      tfBuffer_.transform(targetPose,base2TargetPose,robotFrameID);
      syncTFData(base2TargetPose,base2TargetTF,base2TargetTFMsg,targetFrameID);
//      syncTFData(base2TargetTFMsg,base2TargetTF,base2TargetPose);
////      tfBuffer_.transform(targetPose,targetPose,"base_link");
//      tf2::convert(targetTFMsg.transform,targetTF);
//      tf2::toMsg(targetTF,targetPose.pose);
      ROS_INFO_STREAM("TRANSFORMED BASE->TARGET TF MSG " << base2TargetTFMsg);
      ROS_INFO_STREAM("TRANSFORMED BASE->TARGET TRANSFORM " << transformString(base2TargetTF));
      ROS_INFO_STREAM("TRANSFORMED BASE->TARGET POSE MSG " << poseString(base2TargetPose.pose));

//      ROS_INFO_STREAM("TRANSFORM FOUND BETWEEN base_link and " << targetPose.header.frame_id);
//      robotTFMsg = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0));
//      tf2::convert(robotTFMsg.transform,robotTF);
//      tf2::toMsg(robotTF,robotPose.pose);
//      tf2::convert(robotTFMsg.transform,robotTF);
//      tf2::convert(robotTFMsg,robotPose);
//      tf2::toMsg(robotTF,robotPose.pose);

//      tfBuffer_.transform(projectionPose,projectionPose,"base_link");
//      ROS_INFO_STREAM("TRANSFORMED ROBOT POSE " << projectionPose);
    }
    catch (tf::TransformException const &ex)  {
      ROS_ERROR("%s",ex.what());
      return false;
    }

//    projectionPose.pose.position.x = projectionPose.pose.position.y = projectionPose.pose.orientation.z = 0.0;
//    projectionPose.pose.orientation.w = 1.0;
//    projectionPose.header.stamp = ros::Time(0);
//    targetPose.header.frame_id = projectionPose.header.frame_id = "base_link";
//    ROS_INFO_STREAM("CURRENT ROBOT POSE " << projectionPose);
    base2ProjectionTFMsg.header.frame_id = robotFrameID;
    base2ProjectionTFMsg.child_frame_id = projectionFrameID;
    base2ProjectionTFMsg.header.stamp = ros::Time::now();
    base2ProjectionTFMsg.transform.translation.x = base2ProjectionTFMsg.transform.translation.y = base2ProjectionTFMsg.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    tf2::convert(q,base2ProjectionTFMsg.transform.rotation);

    syncTFData(base2ProjectionTFMsg,base2ProjectionTF,base2ProjectionPose);
    ROS_INFO_STREAM("BASE->PROJECTION TF MSG" << base2ProjectionTFMsg);
//    tf2::convert(projectionTFMsg.transform,projectionTF);
    ROS_INFO_STREAM("BASE->PROJECTION TRANSFORM" << transformString(base2ProjectionTF));
////    tf2::convert(projectionTFMsg,projectionPose);
//    tf2::toMsg(projectionTF,projectionPose.pose);
    ROS_INFO_STREAM("BASE->PROJECTION POSE MSG" << poseString(base2ProjectionPose.pose));


    proj2TargetTF = getProjectionToTargetTF(base2ProjectionTF,base2TargetTF);
    syncTFData(proj2TargetTF,proj2TargetTFMsg,proj2TargetPose,projectionFrameID,targetFrameID);


//    ROS_INFO_STREAM("CURRENT POSE " << targetPose);
    // Distance to goal
//    double goalDist = getDistToGoal(targetPose.pose.position.x, targetPose.pose.position.y);
    double goalDist = proj2TargetTF.getOrigin().length();
    ROS_INFO_STREAM("GETTING DISTANCE TO GOAL: " << goalDist);

    // If within distance tolerance, return true
    if (goalDist < 0.01)
    {
      ROS_INFO_STREAM("WITHIN DISTANCE TOLERANCE. GOAL REACHED");
      return true;
    }

//    // Add initial robot pose to plan
    addtoPlan(base2ProjectionPose,twist);

    while(goalDist > 0.01){

      if(steps > path_steps_){
        break;
      }

      ROS_INFO_STREAM("DIST TO GOAL = " << goalDist << " > 0.01");
//      // Orientation base frame relative to r_
      ROS_INFO_STREAM("GETTING DELTA ANGLE");
//      // @TODO TODO Make sure this is still valid since the reference robotPose is changing as well
      double deltaAngle;
//      deltaAngle = getDeltaAngle(targetPose.pose.position.y, targetPose.pose.position.x);
      deltaAngle = getDeltaAngle(proj2TargetTF);
      ROS_INFO_STREAM("DELTA ANGLE " << deltaAngle);


      ROS_INFO_STREAM("GETTING PHI");
//      // Determine orientation of goal frame relative to r_
      double phi = 69;

      phi = getPhi(proj2TargetTF);
      ROS_INFO_STREAM("PHI " << phi);

      ROS_INFO_STREAM("GETTING DELTA CONTROL");
      // Compute the virtual control
      double deltaControl = getDeltaControl(phi);
      ROS_INFO_STREAM("DELTA CONTROL " << deltaControl);

      // Compute curvature (k)
      ROS_INFO_STREAM("GETTING CURVATURE");
      double curvature = getCurvature(goalDist, deltaAngle, deltaControl, phi);
      ROS_INFO_STREAM("CURVATURE " << curvature);

      // Compute max_velocity based on curvature
      double v = lin_vel_max_ / (1 + beta_ * std::pow(fabs(curvature), lambda_));
      // Limit max velocity based on approaching target (avoids overshoot)
      if (goalDist < 0.5)
      {
        v = std::max(lin_vel_min_, std::min(std::min(goalDist, lin_vel_max_), v));
      }
      else
      {
        v = std::min(lin_vel_max_, std::max(lin_vel_min_, v));
      }

      // Compute angular velocity
      double omega = curvature * v;
      // Bound angular velocity
      double omegaBounded = std::min(omega_max_, std::max(-omega_max_, omega));
      // Make sure that if we reduce w, we reduce v so that curvature is still followed
      if (omega != 0.0) {
        v *= (omegaBounded/omega);
      }

      twist.linear.x = v;
      twist.angular.z = omegaBounded;

      ROS_INFO_STREAM("ADDING POSE TO PLAN" << poseString(base2ProjectionPose.pose));
      addtoPlan(base2ProjectionPose,twist);
//      ROS_INFO_STREAM("STEPPING POSE");
//      stepPose(base2ProjectionPose,twist);
//      ROS_INFO_STREAM("STEPPED POSE TO " << poseString(projectionPose.pose));
//      ROS_INFO_STREAM("UPDATING TARGET POSE");
//      updateTargetPose(projectionPose,targetPose);
//      ROS_INFO_STREAM("TARGET POSE UPDATED TO " << poseString(targetPose.pose));

//      ROS_INFO_STREAM("GETTING DISTANCE BETWEEN UPDATED TARGET AND STEPPED POSE");
//      double oldGoalDist = goalDist;
//      goalDist = getDistBetweenPoses(projectionPose.pose, targetPose.pose);
//      std::ostringstream gdSS;
//      gdSS << std::fixed << std::setprecision(2) << "OLD: " << oldGoalDist << " NEW: " << goalDist;
//      ROS_INFO_STREAM("GOAL DISTANCE " << gdSS.str());

//      ROS_INFO_STREAM("END PATH STEP #" << steps);
      steps++;

////      std::cout << std::endl;
//      std::cout << "-------------------------------------------------";
//      std::cout << std::endl;
    }

//    twist.linear.x = twist.angular.z = 0.0;
//    addtoPlan(origTargetPose,twist);

//    path_pub_.publish(plan_.path);
//    pose_array_pub_.publish(plan_.poseArray);

    std::cout << std::endl << std::endl ;
    std::cout << "###################################################" << std::endl;
    std::cout << "###################################################" << std::endl;

    return true;
  }

  void dockPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
//    ROS_INFO_STREAM("DOCK POSE CALLBACK");
    geometry_msgs::PoseStamped pose = *msg;
//    if(calculateCurrentApproach(pose))
    if(calculatePlan(pose))
    {
      ROS_INFO_STREAM("SUCCESSFULLY GENERATED PATH");
    }
    else {
//      ROS_WARN_STREAM("FAILED TO GENERATE PATH");
    }

  }

  double getDistToGoal(double& x, double& y){
//    ROS_INFO_STREAM("X " << x);
//    ROS_INFO_STREAM("Y " << y);
    double xx = x*x;
//    ROS_INFO_STREAM("XX " << xx);
    double yy = y*y;
//    ROS_INFO_STREAM("YY " << yy);
    double xxyy = xx + yy;
//    ROS_INFO_STREAM("XX+YY " << xxyy);
    double d = std::sqrt(xxyy);
//    ROS_INFO_STREAM("DISTANCE BETWEEN POSES " << d);
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

// ******************************** DELTA ********************************

  double getDeltaAngle(double& dy, double& dx){ return (std::atan2(-dy, dx));}
  void getDeltaAngle(double& dy, double& dx, double& delta){ delta = getDeltaAngle(dy,dx);}

  double getDeltaAngle(tf2::Transform p2tTF){
    double da;
    // True when the target pose passed was in base_link frame
    // X Axis Vector in base_link / projected base_link frame
    tf2::Vector3 vecProjX(1,0,0);
//    ROS_INFO_STREAM("Delta Angle: Robot/Projection X Vector " << vectorString(vecProjX));
    // LOS Vector in base_link / projected base_link frame
    tf2::Vector3 vecLOS = p2tTF.getOrigin();
//    ROS_INFO_STREAM("Delta Angle: Line Of Sight Vector " << vectorString(vecLOS));
    da = vecLOS.angle(vecProjX);
//    ROS_INFO_STREAM("Delta Angle: vecLOS.angle(vecRobotX) " << da);
    return da;
  }
  double getDeltaControl(double& phi){ return (std::atan(-k1_ * phi));}
  void getDeltaControl(double& phi, double& deltaControl){ deltaControl = getDeltaControl(phi);}

// ******************************** PHI ********************************

  double getPhi(tf2::Transform p2tTF){
    double phi;
    // X Axis Vector in base_link / projected base_link frame
    tf2::Vector3 vecProjX(1,0,0);
//    ROS_INFO_STREAM("Phi Angle: Robot/Projection X Vector " << vectorString(vecProjX));
    tf2::Vector3 vecProj2Target = p2tTF.getOrigin();
//    ROS_INFO_STREAM("Phi Angle: Robot/Projection --> Target Vector " << vectorString(vecProj2Target));
    tf2::Vector3 vecTargetX = vecProjX * vecProj2Target;
//    ROS_INFO_STREAM("Phi Angle: Target X Vector =  vecProjX * vecProj2Target" << vectorString(vecTargetX));

    tf2::Vector3 vecTargetXRotated = tf2::quatRotate(p2tTF.getRotation(),vecTargetX);
//    ROS_INFO_STREAM("Phi Angle: Target X Vector Rotated by Q " << vectorString(vecTargetXRotated));

    phi = vecProj2Target.angle(vecTargetXRotated);
//    ROS_INFO_STREAM("Phi Angle: vecProj2Target.angle(vecTargetX) " << phi);

    return phi;
  }


//  double getPhi(tf2::Transform p2tTF){
//    double phi = tf2::getYaw(p2tTF.getRotation());
//    ROS_INFO_STREAM("Phi Angle: tf2::getYaw(p2tTF.getRotation()) " << phi);
//    return phi;
//  }

  double getPhi(geometry_msgs::Quaternion& qMsg, double& delta){
    ROS_INFO_STREAM("Getting Phi from Quaternion and Yaw" << quaternionString(qMsg));
    tf::Quaternion qTF;
//    qTF.normalize();
    tf::quaternionMsgToTF(qMsg, qTF);
    ROS_INFO_STREAM("Quaternion TF" << quaternionString(qTF)); //printTFQuarternion(qTF);
    qTF.normalize();
    ROS_INFO_STREAM("Quaternion TF Normalized" << quaternionString(qTF)); //printTFQuarternion(qTF);
    double phi = angles::normalize_angle(tf::getYaw(qTF) + delta);
    ROS_INFO_STREAM("Finished Getting Phi from Quaternion and Yaw == Phi:" << phi);
    return phi;
  }
  void getPhi(geometry_msgs::Quaternion& qMsg, double& delta, double& phi){
    phi = getPhi(qMsg, delta);
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

  double getCurvature(double& goalDist, double& deltaAngle, double& deltaControl, double& phi){
    ROS_INFO_STREAM("Getting Curvature");
    double k = (-1.0/goalDist * (k2_ * (deltaAngle - deltaControl) + (1 + (k1_/1+((k1_*phi)*(k1_*phi))))*sin(deltaAngle)));
    return k;
  }

  void addtoPlan(geometry_msgs::PoseStamped& ps, geometry_msgs::Twist& twist){
    plan_.path.poses.push_back(ps);
    plan_.poseArray.poses.push_back(ps.pose);
    plan_.twists.push_back(twist);
  }


  void stepPose(geometry_msgs::PoseStamped& poseOld, geometry_msgs::Twist& t){
    ROS_INFO_STREAM("Started Stepping Pose with Twist " << twistString(t));
    geometry_msgs::PoseStamped poseDelta, poseNew;
    poseOld.header.stamp = poseOld.header.stamp + time_step_duration_;

    double yawOld = getYaw(poseOld.pose.orientation), yawNew=0;
    tf2::Quaternion qOld, qDelta, qNew;
    tf2::convert(poseOld.pose.orientation , qOld);



    poseDelta.pose.position.x = (t.linear.x * cos(yawOld)  -  t.linear.y * sin(yawOld)) * time_step_;
    poseDelta.pose.position.y = (t.linear.x * sin(yawOld)  +  t.linear.y * cos(yawOld)) * time_step_;
    double yawDelta = t.angular.z * time_step_;
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

  void updateTargetPose(geometry_msgs::PoseStamped& r, geometry_msgs::PoseStamped& t){
    ROS_INFO_STREAM("Updating Target Pose");
    t.header.stamp = r.header.stamp;
    t.pose.position.x -= r.pose.position.x;
    t.pose.position.y -= r.pose.position.y;
    double rYaw = getYaw(r.pose.orientation);
    double tYaw = getYaw(t.pose.orientation);
    tYaw -= rYaw;
    t.pose.orientation = getQuaternion(tYaw);
//    t.pose.orientation.z -= r.pose.orientation.z;
//    t.pose.orientation.w -= r.pose.orientation.w;
//    ROS_INFO_STREAM("Target Pose: Quaternion Msg" << t.pose.orientation);
//    tf::Quaternion qTF;
//    tf::quaternionMsgToTF(t.pose.orientation, qTF);
//    ROS_INFO_STREAM("Target Pose: Quaternion TF" << qTF);
//    qTF.normalize();
//    ROS_INFO_STREAM("Target Pose: Quaternion TF Normalized" << qTF);
//    tf::quaternionTFToMsg(qTF, t.pose.orientation);
//    ROS_INFO_STREAM("Finished Updating Target Pose");
  }

  tf2::Transform getProjectionToTargetTF(tf2::Transform& base2Proj, tf2::Transform& base2Target){
    // AtoC = AtoB * BtoC;
    tf2::Transform proj2Base, proj2Target;
    proj2Base = base2Proj.inverse();
    proj2Target = proj2Base * base2Target;
    ROS_INFO_STREAM("GETTING TF FROM PROJECTION TO TARGET " << transformString(proj2Target));
    return proj2Target;
  }

  void syncTFData(tf2::Transform& tf2, geometry_msgs::TransformStamped& tfMsg, geometry_msgs::PoseStamped& poseStamped, std::string frameID, std::string childFrameID)
  {
    ROS_INFO_STREAM("SYNCING TF --> TF MSF and POSE" << transformString(tf2));
    // tf2 to tfMSG
    tf2::convert(tf2,tfMsg.transform);
    tfMsg.header.frame_id = frameID;
    tfMsg.child_frame_id = childFrameID;
    tfMsg.header.stamp = ros::Time::now();
    ROS_INFO_STREAM("SYNCING TF MSG FROM TF " << tfMsg);
    // tf2 to Pose
    tf2::toMsg(tf2,poseStamped.pose);
    poseStamped.header.frame_id = frameID;
    poseStamped.header.stamp = tfMsg.header.stamp;
    ROS_INFO_STREAM("SYNCING POSE FROM TF " << poseStamped);
  }

  void syncTFData(geometry_msgs::TransformStamped& tfMsg, tf2::Transform& tf2, geometry_msgs::PoseStamped& poseStamped)
  {
    ROS_INFO_STREAM("SYNCING TF MSG --> TF and POSE" << tfMsg);
    // tfMSG to tf2
    tf2::convert(tfMsg.transform,tf2);
    ROS_INFO_STREAM("SYNCING TF FROM TF MSG" << transformString(tf2));
    // tf2 to Pose
    tf2::toMsg(tf2,poseStamped.pose);
    poseStamped.header.stamp = tfMsg.header.stamp;
    poseStamped.header.frame_id = tfMsg.header.frame_id;
    ROS_INFO_STREAM("SYNCING POSE FROM TF MSG" << poseStamped);
  }

  void syncTFData(geometry_msgs::PoseStamped& poseStamped, tf2::Transform& tf2, geometry_msgs::TransformStamped& tfMsg, std::string childFrameID)
  {
    ROS_INFO_STREAM("SYNCING POSE --> TF and TF MSG" << poseStamped);
    // pose to tf2
    tf2::fromMsg(poseStamped.pose,tf2);
    ROS_INFO_STREAM("SYNCING TF FROM POSE" << transformString(tf2));
    // tf2 to TFMsg
    tf2::convert(tf2,tfMsg.transform);
    tfMsg.header.frame_id = poseStamped.header.frame_id;
    tfMsg.child_frame_id = childFrameID;
    tfMsg.header.stamp = poseStamped.header.stamp;
    ROS_INFO_STREAM("SYNCING TF MSG FROM POSE" << tfMsg);
  }

};

#endif // POSECONTROLLERNODE_H
