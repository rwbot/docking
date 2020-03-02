#ifndef POSECONTROLLERNODE_H
#define POSECONTROLLERNODE_H

#include <docking/Headers.h>
#include <docking/PoseControllerConfig.h>
// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
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
  }
  ~PoseControllerNode(){}

  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;  // Publisher of commands
  ros::Publisher path_pub_;  // Publisher of paths
  ros::Publisher pose_array_pub_;  // Publisher of pose array
  ros::Subscriber dockPoseSub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
//  tf::TransformListener tfListener_;
  geometry_msgs::PoseArray poseArray_;

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
  double lin_vel_min;
  double lin_vel_max;
  int path_steps_;
  double time_step_;

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
    lin_vel_min = config.v;
    lin_vel_max = config.v;
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

  ///////////////// BEGIN CALCULATE CURRENT APPROACH /////////////////
  /// \brief calculateCurrentApproach
  /// \param targetPose
  /// \return
  ///

  bool calculateCurrentApproach(geometry_msgs::PoseStamped& targetPose){

    geometry_msgs::PoseStamped pose = targetPose;
    pose.header.stamp = ros::Time(0);
//    {
//      double theta = angles::normalize_angle(tf::getYaw(q));
//      // If the target has an invalid orientation then don't approach it.
//      if (!std::isfinite(theta))
//      {
//        ROS_ERROR_STREAM_NAMED("controller", "Invalid approach target for docking.");
//        return true;
//      }
//      pose.pose.position.x += cos(theta) * -dist_;
//      pose.pose.position.y += sin(theta) * -dist_;
//    }

//     Transform target into base frame
    try
    {
//      tfListener_.transformPose("base_link", targetPose, pose);
      tfBuffer_.transform(targetPose,pose,"base_link");
//      ROS_INFO_STREAM("TRANSFORM FOUND BETWEEN base_link and " << targetPose.header.frame_id);
    }
    catch (tf::TransformException const &ex)
    {
//      ROS_WARN_STREAM_THROTTLE(1.0, "Couldn't get transform from dock to base_link");
      ROS_ERROR("%s",ex.what());
      return false;
    }

    // Distance to goal
    double goalDist_ = getDistToGoal(pose.pose.position.x, pose.pose.position.y);

    // If within distance tolerance, return true
    if (goalDist_ < 0.01)
    {
      ROS_INFO_STREAM("WITHIN DISTANCE TOLERANCE. GOAL REACHED");
      return true;
    }

    // Orientation base frame relative to r_
    double deltaAngle = getDeltaAngle(pose.pose.position.y, pose.pose.position.x);

    // Determine orientation of goal frame relative to r_
    double phi = getPhi(pose.pose.orientation, deltaAngle);

    // Compute the virtual control
//    double a = atan(-k1_ * phi);
    double deltaControl = getDeltaControl(phi);

    // Compute curvature (k)
//    double k = -1.0/r * (k2_ * (deltaAngle - deltaControl) + (1 + (k1_/1+((k1_*phi)*(k1_*phi))))*sin(deltaAngle));
    double curvature = getCurvature(goalDist_, deltaAngle, deltaControl, phi);

    // Compute max_velocity based on curvature
    double v = lin_vel_max / (1 + beta_ * std::pow(fabs(curvature), lambda_));
    // Limit max velocity based on approaching target (avoids overshoot)
    if (goalDist_ < 0.75)
    {
      v = std::max(lin_vel_min, std::min(std::min(goalDist_, lin_vel_max), v));
    }
    else
    {
      v = std::min(lin_vel_max, std::max(lin_vel_min, v));
    }

    // Compute angular velocity
    double omega = curvature * v;
    // Bound angular velocity
    double omegaBounded = std::min(omega_max_, std::max(-omega_max_, omega));
    // Make sure that if we reduce w, we reduce v so that kurvature is still followed
    if (omega != 0.0)
    {
      v *= (omegaBounded/omega);
    }

    geometry_msgs::Twist twist;
    twist.linear.x = v;
    twist.angular.z = omegaBounded;

    nav_msgs::Path plan;
    geometry_msgs::PoseArray poseArray;
    projectCurrentOmega(targetPose, twist, phi, plan, poseArray);

    return false;
  }
///////////////// END CALCULATE CURRENT APPROACH  /////////////////


  void projectCurrentOmega(geometry_msgs::PoseStamped& goalPose, geometry_msgs::Twist& twist, double& phi, nav_msgs::Path& plan, geometry_msgs::PoseArray& poseArray)
  {
    // Create debugging view of path
    plan.header.stamp = poseArray.header.stamp = ros::Time::now();
    plan.header.frame_id = poseArray.header.frame_id = "base_link";
    // Add origin
    geometry_msgs::PoseStamped path_pose;
    path_pose.header.frame_id = "base_link";
    path_pose.pose.orientation.w = 1.0;
    plan.poses.push_back(path_pose);
    poseArray.poses.push_back(path_pose.pose);
    double yaw = 0.0;
    for (int i = 0; i < path_steps_; i++)  // 2 sec
    {
      path_pose.pose.position.x += time_step_ * twist.linear.x * cos(yaw);
      path_pose.pose.position.y += time_step_ * twist.linear.x * sin(yaw);
      yaw += time_step_ * twist.angular.z;
      path_pose.pose.orientation.z = -sin(phi/2.0);
      path_pose.pose.orientation.w = cos(phi/2.0);

      double dx = path_pose.pose.position.x - goalPose.pose.position.x;
      double dy = path_pose.pose.position.y - goalPose.pose.position.y;
      if ((dx * dx + dy * dy) < 0.005)
      {
        break;
      }

      plan.poses.push_back(path_pose);
      poseArray.poses.push_back(path_pose.pose);
    }
    // Push goal pose onto path
    plan.poses.push_back(goalPose);
    poseArray.poses.push_back(goalPose.pose);
    // Publish path
    path_pub_.publish(plan);
    pose_array_pub_.publish(poseArray);
  }

  void dockPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
//    ROS_INFO_STREAM("DOCK POSE CALLBACK");
    geometry_msgs::PoseStamped pose = *msg;
    if(calculateCurrentApproach(pose))
    {
      ROS_INFO_STREAM("SUCCESSFULLY GENERATED PATH");
    }
    else {
//      ROS_WARN_STREAM("FAILED TO GENERATE PATH");
    }

  }

  double getDistToGoal(double& x, double& y){  return (std::sqrt(x*x + y*y));}
  void getDistToGoal(double& x, double& y, double& goalDist){  goalDist = getDistToGoal(x,y);}


  double getDeltaAngle(double& dy, double& dx){ return (std::atan2(-dy, dx));}
  void getDeltaAngle(double& dy, double& dx, double& delta){ delta = getDeltaAngle(dy,dx);}

  double getDeltaControl(double& phi){ return (std::atan(-k1_ * phi));}
  void getDeltaControl(double& phi, double& deltaControl){ deltaControl = getDeltaControl(phi);}

  double getPhi(geometry_msgs::Quaternion& qMsg, double& delta){
    tf::Quaternion qTF;
    tf::quaternionMsgToTF(qMsg, qTF);
    return (angles::normalize_angle(tf::getYaw(qTF) + delta));
  }
  void getPhi(geometry_msgs::Quaternion& qMsg, double& delta, double& phi){
    phi = getPhi(qMsg, delta);
  }


  double getCurvature(double& goalDist, double& deltaAngle, double& deltaControl, double& phi){
    double k = (-1.0/goalDist * (k2_ * (deltaAngle - deltaControl) + (1 + (k1_/1+((k1_*phi)*(k1_*phi))))*sin(deltaAngle)));
    return k;
  }

};

#endif // POSECONTROLLERNODE_H
