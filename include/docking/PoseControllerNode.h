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
#include <math.h>

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
    startPub();
    startPoseSub();
    initGlobals();
  }
  ~PoseControllerNode(){
    cmd_vel_pub_.publish(zeroTwist_);
  }

  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;  // Publisher of commands
  ros::Publisher path_pub_;  // Publisher of paths
  ros::Publisher pose_array_pub_;  // Publisher of pose array
  ros::Publisher twist_array_pub_;  // Publisher of twist array
  ros::Subscriber dockPoseSub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  docking::Plan plan_;

  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<docking::PoseControllerConfig> dr_srv_;
  //! Dynamic Reconfig Variables
  std::string dock_frame_;
  double omega_max_;
  double kPhi_;  // ratio in change of theta to rate of change in r
  double kDelta_;  // speed at which we converge to slow system
  double curvature_max_;
  double beta_;  // how fast velocity drops as k increases
  double lambda_;  // ??
  double lin_vel_min_;
  double lin_vel_max_;
  bool use_steps_;
  int path_steps_;
  double time_step_;
  ros::Duration time_step_duration_;
  bool publish_twist_;
  double goal_dist_tolerance_;
  double goal_orientation_tolerance_;
  std::string dock_pose_topic_;
  std::string dock_gazebo_pose_topic_;
  bool use_calculated_pose_;
  int max_steps_;
//  int frequency_;
//  ros::Rate frequencyRate_;
  double entrance_dist_;

  //! Control System Variables
  double goalDist_;
  double deltaAngle_;
  double phi_;
  double deltaControl_;
  double curvature_;
  double v_;
  double omega_;
  double omegaBounded_;
  geometry_msgs::Twist zeroTwist_;


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
    kPhi_ = config.kPhi;
    kDelta_ = config.kDelta;
    curvature_max_ = config.curvature_max;
    beta_ = config.beta;
    lambda_ = config.lambda;
    lin_vel_min_ = config.v;
    lin_vel_max_ = config.v;
    path_steps_ = config.path_steps;
    time_step_ = config.time_step;
    publish_twist_ = config.publish_twist;
    use_steps_ = config.use_steps;
    goal_orientation_tolerance_ = config.goal_orientation_tolerance;
    goal_dist_tolerance_ = config.goal_dist_tolerance;
    dock_pose_topic_ = config.dock_pose_topic;
    max_steps_ = config.max_steps;
    dock_gazebo_pose_topic_ = config.dock_gazebo_pose_topic;
//    frequency_ = config.frequency;
    entrance_dist_ = config.entrance_dist;
    if(use_calculated_pose_ != config.use_calculated_pose){
      use_calculated_pose_ = config.use_calculated_pose;
      startPoseSub();
    }
  }

  void startPub() {
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("docking/path", 10);
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("docking/pose_array", 10);
    twist_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("docking/twist_array", 10);
  }

//  void startPoseSub(std::string& dockPoseTopic)
  void startPoseSub()
  {
    if(use_calculated_pose_){
      ROS_WARN_STREAM("PoseControllerNode: Subscribing to " << dock_pose_topic_);
      dockPoseSub_ = nh_.subscribe(dock_pose_topic_, 1, &PoseControllerNode::dockPoseCallback, this);
    } else {
      ROS_INFO_STREAM("PoseControllerNode: Subscribing to " << dock_gazebo_pose_topic_);
      dockPoseSub_ = nh_.subscribe(dock_gazebo_pose_topic_, 1, &PoseControllerNode::dockPoseCallback, this);
    }
  }

  void initGlobals(){
    ros::Duration time_step_duration(time_step_);
    time_step_duration_ = time_step_duration;
//    ros::Rate frequencyRate(frequency_);
//    frequencyRate_ = frequencyRate;
    plan_.header.frame_id = plan_.path.header.frame_id = plan_.poseArray.header.frame_id = "base_link";
    zeroTwist_.linear.x = zeroTwist_.angular.z = 0.0;
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
    geometry_msgs::Twist twist, currentTwist;
    int steps = 0;

    ROS_INFO_STREAM("**************** BEGINNING CALCULATING PLAN ****************");
    tf2::Transform base2ProjectionTF, robotTF, base2TargetTF, base2EntranceTF, proj2TargetTF, deltaTF;
    geometry_msgs::TransformStamped base2ProjectionTFMsg, robotTFMsg, base2TargetTFMsg, base2EntranceTFMsg, proj2TargetTFMsg, deltaTFMsg, tempTFMsg;
    geometry_msgs::PoseStamped base2ProjectionPose, robotPose, base2TargetPose, base2EntrancePose, proj2TargetPose, deltaTFPose;
    std::string projectionFrameID="projection", robotFrameID="base_link", targetFrameID="target", entranceFrameID="entrance";

//    Init Base to Projection
    base2ProjectionTFMsg.header.frame_id = robotFrameID;
    base2ProjectionTFMsg.child_frame_id = projectionFrameID;
    base2ProjectionTFMsg.header.stamp = ros::Time::now();
    base2ProjectionTFMsg.transform.translation.x = base2ProjectionTFMsg.transform.translation.y = base2ProjectionTFMsg.transform.translation.z = 0.0;
    tf2::Quaternion qProjection;
    qProjection.setRPY(0, 0, 0);
    tf2::convert(qProjection,base2ProjectionTFMsg.transform.rotation);
    syncTFData(base2ProjectionTFMsg,base2ProjectionTF,base2ProjectionPose);
//    ROS_INFO_STREAM("BASE->PROJECTION TF MSG" << transformString(base2ProjectionTFMsg));
//    ROS_INFO_STREAM("BASE->PROJECTION TRANSFORM" << transformString(base2ProjectionTF));
//    ROS_INFO_STREAM("BASE->PROJECTION POSE MSG" << poseString(base2ProjectionPose.pose));

    // Init Base to Entrance
    base2EntranceTFMsg.header.frame_id = robotFrameID;
    base2EntranceTFMsg.child_frame_id = entranceFrameID;
    base2EntranceTFMsg.header.stamp = ros::Time::now();
    base2EntranceTFMsg.transform.translation.x = entrance_dist_;
    base2EntranceTFMsg.transform.translation.y = base2EntranceTFMsg.transform.translation.z = 0.0;
    tf2::Quaternion qEntrance;
    qEntrance.setRPY(0, 0, 0);
    tf2::convert(qEntrance,base2ProjectionTFMsg.transform.rotation);
    syncTFData(base2EntranceTFMsg,base2EntranceTF,base2EntrancePose);


    ROS_INFO_STREAM("ORIGINAL TARGET POSE MSG" << poseString(targetPose));
    double targetYaw = tf2::getYaw(targetPose.pose.orientation);
    targetPose.pose.position.x -= entrance_dist_ * cos(targetYaw);
    targetPose.pose.position.y -= entrance_dist_ * sin(targetYaw);
    ROS_INFO_STREAM("TARGET POSE MSG + ENTRANCE DISTANCE" << poseString(targetPose));


    // Transform targetPose into base_link frame
    try {
      ROS_INFO_STREAM("TRANSFORMING TARGET POSE FROM " << targetPose.header.frame_id << " TO " << robotFrameID);
      tfBuffer_.transform(targetPose,base2TargetPose,robotFrameID);
      base2TargetPose.pose.position.z = 0;
      syncTFData(base2TargetPose,base2TargetTF,base2TargetTFMsg,targetFrameID);
      ROS_INFO_STREAM("TRANSFORMED BASE->TARGET TF MSG " << transformString(base2TargetTFMsg));
//      ROS_INFO_STREAM("TRANSFORMED BASE->TARGET TRANSFORM " << transformString(base2TargetTF));
      ROS_INFO_STREAM("TRANSFORMED BASE->TARGET POSE MSG " << poseString(base2TargetPose.pose));

//      ROS_INFO_STREAM("TRANSFORM FOUND BETWEEN base_link and " << targetPose.header.frame_id);
//      robotTFMsg = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf::TransformException const &ex)  {
      ROS_ERROR("%s",ex.what());
      return false;
    }
    std::cout << std::endl;

    proj2TargetTF = getProjectionToTargetTF(base2ProjectionTF,base2TargetTF);
    syncTFData(proj2TargetTF,proj2TargetTFMsg,proj2TargetPose,projectionFrameID,targetFrameID);

    // Distance to goal
    double goalDist = proj2TargetTF.getOrigin().length();
    ROS_INFO_STREAM("GETTING DISTANCE TO GOAL: " << goalDist);
    double deltaAngle = 1.0;
    // If within distance tolerance, return true
    if((goalDist < goal_dist_tolerance_) && (fabs(deltaAngle) < goal_orientation_tolerance_))
    {
      ROS_WARN_STREAM("WITHIN DISTANCE TOLERANCE. GOAL REACHED");
      return true;
    }

    // Add initial robot pose to plan
    addtoPlan(base2ProjectionPose,zeroTwist_);

    while((goalDist > goal_dist_tolerance_) || (fabs(deltaAngle) > goal_orientation_tolerance_)){

      if(use_steps_){
        if(steps > path_steps_){
          ROS_WARN_STREAM("FINISHED " << steps << " STEPS");
          break;
        }
      }

      if(steps >= max_steps_){
        ROS_WARN_STREAM("HIT MAXIMUM STEPS " << steps );
        break;
      }

      ROS_INFO_STREAM("DIST TO GOAL = " << goalDist << " > " << goal_dist_tolerance_);
//      // Orientation base frame relative to r_

      deltaAngle = getDeltaAngle(proj2TargetPose.pose.position.y, proj2TargetPose.pose.position.x);
//      deltaAngle = getDeltaAngle(proj2TargetTF);
      ROS_INFO_STREAM("GETTING DELTA ANGLE " << deltaAngle);


//      // Determine orientation of goal frame relative to r_
//      phi = getPhi(proj2TargetTF);
      double phi = tf2::getYaw(proj2TargetTF.getRotation());
      ROS_INFO_STREAM("PHI =  YAW (" << phi << ") + delta (" << deltaAngle << ") = " << (phi + deltaAngle));
      phi += deltaAngle;
//      ROS_INFO_STREAM("GETTING PHI " << phi);



      // Compute the virtual control
      double deltaControl = getDeltaControl(phi);
      ROS_INFO_STREAM("GETTING DELTA CONTROL " << deltaControl);

      // Compute curvature (k)
//      double curvature = getCurvature(goalDist, deltaAngle, deltaControl, phi);
//      ROS_INFO_STREAM("GETTING CURVATURE " << curvature);

      double omega = calculateOmega(goalDist, deltaAngle, deltaControl, phi);
      ROS_INFO_STREAM("CALCULATING OMEGA DIRECTLY " << omega);

//      // Compute max_velocity based on curvature
      double v = lin_vel_max_ / (1 + beta_ * std::pow(fabs(curvature), lambda_));
      ROS_INFO_STREAM("CALCULATING V BASED ON CURVATURE " << v);
//      // Limit max velocity based on approaching target (avoids overshoot)
//      if (goalDist < 0.5)
//      {
//        v = std::max(lin_vel_min_, std::min(std::min(goalDist, lin_vel_max_), v));
//        ROS_INFO_STREAM("GOAL LESS THAN 0.5 SETTING MAX V TO " << v);
//      }
//      else
//      {
//        v = std::min(lin_vel_max_, std::max(lin_vel_min_, v));
//        ROS_INFO_STREAM("GOAL GREATER THAN 0.5 SETTING MIN V TO " << v);
//      }
//      // Compute angular velocity
//      double omega = curvature * v;
//      ROS_INFO_STREAM("CALCULATING OMEGA BASED ON CURVATURE " << omega);
//      // Bound angular velocity
      double omegaBounded = std::min(omega_max_, std::max(-omega_max_, omega));
      ROS_INFO_STREAM("BOUNDING OMEGA TO  " << omegaBounded);
//      // Make sure that if we reduce w, we reduce v so that curvature is still followed
//      if (omega != 0.0) {
//        ROS_INFO_STREAM("OMEGA NOT ZERO ");
//        ROS_INFO_STREAM("V = " << v << " * (" << omegaBounded << " / " << omega << ") = " << v * (omegaBounded/omega));
//        v *= (omegaBounded/omega);
//      }


      twist.angular.z = omegaBounded;
//      twist.linear.x = v;
      twist.linear.x = lin_vel_min_;
      ROS_INFO_STREAM("TWIST: " << twistString(twist));
      if(goalDist < entrance_dist_){
        twist.linear.x *= 0.25;
        twist.angular.z *= 0.25;
        ROS_INFO_STREAM("Approaching Target - Twist Scaled Down To " << twistString(twist));
      }
      if(steps == 0){
        currentTwist = twist;
      }


//      ROS_INFO_STREAM("ADDING POSE TO PLAN" << poseString(base2ProjectionPose.pose));
//      addtoPlan(base2ProjectionPose,twist);

//      ROS_INFO_STREAM("STEPPING POSE");
      deltaTFMsg = base2ProjectionTFMsg;
//      syncTFData(deltaTFMsg,deltaTF,deltaTFPose);
      deltaTF = getDeltaTF(base2ProjectionTF,twist);
      syncTFData(deltaTF,deltaTFMsg,deltaTFPose,robotFrameID,projectionFrameID);
      deltaTFMsg.header.stamp = base2ProjectionTFMsg.header.stamp + time_step_duration_;
      tempTFMsg = base2ProjectionTFMsg;
//      tf2::doTransform(base2ProjectionPose, base2ProjectionPose, deltaTFMsg) ;
      base2ProjectionTF = base2ProjectionTF * deltaTF;
      syncTFData(base2ProjectionTF,base2ProjectionTFMsg,base2ProjectionPose,robotFrameID,projectionFrameID);
//      stepPose(base2ProjectionPose,twist);
      ROS_INFO_STREAM("STEPPED  " << poseString(base2ProjectionPose.pose));



      ROS_INFO_STREAM("GETTING DISTANCE BETWEEN UPDATED TARGET & STEPPED POSE");
      double oldGoalDist = goalDist;
      proj2TargetTF = getProjectionToTargetTF(base2ProjectionTF,base2TargetTF);
      syncTFData(proj2TargetTF,proj2TargetTFMsg,proj2TargetPose,projectionFrameID,targetFrameID);
      goalDist = proj2TargetTF.getOrigin().length();
      std::ostringstream gdSS;
      gdSS << std::fixed << std::setprecision(4) << "OLD: " << oldGoalDist << " NEW: " << goalDist;
      ROS_INFO_STREAM("GOAL DISTANCE " << gdSS.str());

      ROS_INFO_STREAM("END PATH STEP #" << steps);

      ROS_INFO_STREAM("ADDING POSE TO PLAN" << poseString(base2ProjectionPose.pose));
      addtoPlan(base2ProjectionPose,twist);

      if(goalDist > oldGoalDist){
        ROS_WARN_STREAM("STEPPED POSE INCREASING GOAL DIST ERROR");
        base2ProjectionTFMsg = tempTFMsg;
        syncTFData(base2ProjectionTFMsg, base2ProjectionTF, base2ProjectionPose);
//        break;
      }


      if(goalDist <= goal_dist_tolerance_){
        ROS_WARN_STREAM("WITHIN DISTANCE TOLERANCE");
        if (fabs(deltaAngle) <= goal_orientation_tolerance_){
          ROS_WARN_STREAM("WITHIN ANGLE TOLERANCE");
          break;
        }
      }


      std::cout << "___________________________________________________________________________________________________" << std::endl;
      std::cout << "___________________________________________________________________________________________________" << std::endl << std::endl;
      steps++;

////      std::cout << std::endl;
//      std::cout << "-------------------------------------------------";
//      std::cout << std::endl;
    }

    ROS_WARN_STREAM("COMPLETED TRAJECTORY CALCULATION ");

    if(publish_twist_){
      ROS_INFO_STREAM("Publish Twist " << twistString(currentTwist));
      cmd_vel_pub_.publish(currentTwist);
    } else {
      cmd_vel_pub_.publish(zeroTwist_);
    }
    addtoPlan(base2TargetPose,zeroTwist_);

    path_pub_.publish(plan_.path);
    pose_array_pub_.publish(plan_.poseArray);

    std::cout << std::endl << std::endl ;
    std::cout << "######################################################################################################" << std::endl;
    std::cout << "######################################################################################################" << std::endl;
    std::cout << "######################################################################################################" << std::endl;
    std::cout << std::endl << std::endl ;

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



// ******************************** DELTA ********************************

  double getDeltaAngle(double& dy, double& dx){ return (std::atan2(-dy, dx));}
  void getDeltaAngle(double& dy, double& dx, double& delta){ delta = getDeltaAngle(dy,dx);}

  double getDeltaAngle(tf2::Transform p2tTF){
    tf2::Transform tfLOS, tfTX;
    double da;
    // True when the target pose passed was in base_link frame
    // X Axis Vector in base_link / projected base_link frame
    tf2::Vector3 vecProjX(1,0,0);
    ROS_INFO_STREAM("Delta Angle: Robot/Projection X Vector " << vectorString(vecProjX));
    // LOS Vector in base_link / projected base_link frame
    tf2::Vector3 vecLOS = p2tTF.getOrigin();
    ROS_INFO_STREAM("Delta Angle: Line Of Sight Vector " << vectorString(vecLOS));
    da = vecLOS.angle(vecProjX);
    ROS_INFO_STREAM("Delta Angle: vecLOS.angle(vecRobotX) " << da);

    da = tf2::tf2Angle(vecLOS,vecProjX);
    ROS_INFO_STREAM("Delta Angle: tf2::tf2Angle(vecLOS,vecProjX) " << da);

    da = tf2::getYaw(p2tTF.getRotation());
    ROS_INFO_STREAM("DELTA Angle: tf2::getYaw(p2tTF.getRotation()) " << da);

//    tfLOS.setOrigin(vecLOS);
//    tfTX.setOrigin(vecProjX);

//    da = tfLOS.getRotation().angleShortestPath(tfTX.getRotation());
//    ROS_INFO_STREAM("Delta Angle: tfLOS.getRotation().angleShortestPath(tfTX.getRotation()) " << da);

    return da;
  }
  double getDeltaControl(double& phi){
    ROS_INFO_STREAM("GETTING DELTA CONTROL = atan(- " << kPhi_ << " * " << phi << ") = atan(- " << kPhi_ * phi << ") " << " = " << (std::atan(-kPhi_ * phi)));
    return (std::atan(-kPhi_ * phi));}
  void getDeltaControl(double& phi, double& deltaControl){ deltaControl = getDeltaControl(phi);}

// ******************************** PHI ********************************

  double getPhi(tf2::Transform p2tTF){
    tf2::Transform tfPT, tfTX;
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
    ROS_INFO_STREAM("Phi Angle: vecProj2Target.angle(vecTargetX) " << phi);

    tfPT.setOrigin(vecProj2Target);
    tfTX.setOrigin(vecTargetXRotated);

//    phi = tfPT.getRotation().angleShortestPath(tfTX.getRotation());
//    ROS_INFO_STREAM("Phi Angle: tfPT.getRotation().angleShortestPath(tfTX.getRotation()) " << phi);

//    phi = tfTX.getRotation().angleShortestPath(tfPT.getRotation());
//    ROS_INFO_STREAM("Phi Angle: tfTX.getRotation().angleShortestPath(tfPT.getRotation()) " << phi);

    return phi;
  }

//  double getPhi(tf2::Transform p2tTF){
//    double phi = tf2::getYaw(p2tTF.getRotation());
//    ROS_INFO_STREAM("Phi Angle: tf2::getYaw(p2tTF.getRotation()) " << phi);
//    return phi;
//  }




  double getCurvature(double& goalDist, double& deltaAngle, double& deltaControl, double& phi){
    ROS_INFO_STREAM("Getting Curvature");
    double k, z, left, right, rightDenom;
    z = getZ(deltaAngle, deltaControl);

    left = kDelta_ * z;
    ROS_INFO_STREAM("GETTING LEFT = " << kDelta_ << " * " << z << " = " << left);

    rightDenom = 1 + ( (kPhi_*phi)*(kPhi_*phi) );
    ROS_INFO_STREAM("GETTING RIGHT DENOM = 1 + (" << (kPhi_*phi) << " * " << (kPhi_*phi) << ") = " << rightDenom);

    right = kPhi_ / rightDenom;
    ROS_INFO_STREAM("GETTING RIGHT = " << kPhi_ << " / " << rightDenom << " = " << right);
    ROS_INFO_STREAM("GETTING RIGHT = " << 1 << " + " << right << " = " << (1 + right));
    right = 1 + right;
    ROS_INFO_STREAM("GETTING RIGHT = " << right << " * sin(" << deltaAngle << ") = " << right << " * " << sin(deltaAngle) << " = " << (right * sin(deltaAngle)));
    right = right * sin(deltaAngle);


    k = left + right;
    ROS_INFO_STREAM("GETTING CURVATURE = " << left << " + " << right << " = " << k);
    ROS_INFO_STREAM("GETTING CURVATURE = " << k << " * (-1) = " << ((-1) * k) );
    k = (-1) * k;
    ROS_INFO_STREAM("GETTING CURVATURE = " << k << " / " << goalDist << " = " << (k / goalDist));
    k = k / goalDist;
//    k = (-1.0/goalDist * (kDelta_ * (deltaAngle - deltaControl) + (1 + (kPhi_/1+((kPhi_*phi)*(kPhi_*phi))))*sin(deltaAngle)));
    return k;
  }

  double calculateOmega(double& goalDist, double& deltaAngle, double& deltaControl, double& phi){
    ROS_INFO_STREAM("Calculating OMEGA with Constant Linear Velocity ->" << lin_vel_min_);
    double omega, z, left, right, rightDenom;
    z = getZ(deltaAngle, deltaControl);

    left = kDelta_ * z;
    ROS_INFO_STREAM("GETTING LEFT = " << kDelta_ << " * " << z << " = " << left);

    rightDenom = 1 + ( (kPhi_*phi)*(kPhi_*phi) );
//    ROS_INFO_STREAM("GETTING RIGHT DENOM = 1 + (" << (kPhi_*phi) << " * " << (kPhi_*phi) << ") = " << rightDenom);

    right = kPhi_ / rightDenom;
//    ROS_INFO_STREAM("GETTING RIGHT = " << kPhi_ << " / " << rightDenom << " = " << right);
//    ROS_INFO_STREAM("GETTING RIGHT = " << 1 << " + " << right << " = " << (1 + right));
    right = 1 + right;
//    ROS_INFO_STREAM("GETTING RIGHT = " << right << " * sin(" << deltaAngle << ") = " << right << " * " << sin(deltaAngle) << " = " << (right * sin(deltaAngle)));
    right = right * sin(deltaAngle);


    omega = left + right;
    ROS_INFO_STREAM("GETTING OMEGA = " << left << " + " << right << " = " << omega);
//    ROS_INFO_STREAM("GETTING OMEGA = " << omega << " * (-1) = " << ((-1) * omega) );
    omega = (-1) * omega;
//    ROS_INFO_STREAM("GETTING OMEGA = " << omega << " / " << goalDist << " = " << (omega / goalDist));
    omega = omega / goalDist;
    ROS_INFO_STREAM("GETTING OMEGA = " << omega << " * " << lin_vel_min_ << " = " << (omega *lin_vel_min_));
    omega = omega * lin_vel_min_;

    return omega;
  }

  double getZ(double& deltaAngle, double& deltaControl){
    double z = deltaAngle - deltaControl;
    ROS_INFO_STREAM("GETTING Z = " << deltaAngle << " - " << deltaControl << " = " << z);
    return z;
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

  tf2::Transform getDeltaTF(tf2::Transform curTF, geometry_msgs::Twist& t){
    tf2::Transform deltaTF;
    ROS_INFO_STREAM("Started Stepping Pose with Twist " << twistString(t));

    tf2::Quaternion qOld, qDelta, qNew;
    tf2::convert(curTF.getRotation(), qOld);
    double yawOld = tf2::getYaw(qOld);
    double deltaX, deltaY, deltaYaw, rho;
    rho = t.linear.x * time_step_;
    deltaYaw = t.angular.z * time_step_;

//    deltaX = (t.linear.x * cos(deltaYaw)  -  t.linear.y * sin(deltaYaw)) * time_step_;
//    deltaY = (t.linear.x * sin(deltaYaw)  +  t.linear.y * cos(deltaYaw)) * time_step_;
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


};

#endif // POSECONTROLLERNODE_H
