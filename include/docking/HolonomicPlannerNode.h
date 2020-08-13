#ifndef HOLONOMICPLANNERNODE_H
#define HOLONOMICPLANNERNODE_H

#include <docking/Headers.h>
#include <docking/HolonomicPlannerNodeConfig.h>

class HolonomicPlannerNode
{
public:
  HolonomicPlannerNode(ros::NodeHandle &nh) :
    nh_(nh), tfListener_(tfBuffer_)
  {
    //    nh_ = nh;
    ROS_INFO_STREAM("INITIALIZING HOLONOMIC PLANNER NODE OBJECT");
    ROS_INFO_STREAM("STARTING DYNAMIC RECONFIGURE SERVER");
    startDynamicReconfigureServer();
    startActivationSub();
    startPub();
    startOdomSub();
    startPoseSub();
    initGlobals();
    initParams();
  }
  ~HolonomicPlannerNode(){ cmd_vel_pub_.publish(zeroTwist_); }


  ros::NodeHandle nh_;
  ros::Publisher path_pub_;  // Publisher of paths
  ros::Publisher pose_array_pub_;  // Publisher of pose array
  ros::Publisher pose_executed_array_pub_;  // Publisher of pose array
  ros::Publisher cmd_vel_pub_;  // Publisher of twist array
  ros::Publisher plan_pub_; // Publisher of docking::Plan msg
  ros::Subscriber dockPoseSub_;
  ros::Subscriber odomSub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  docking::Plan plan_;

  ros::Subscriber activationSub_;
  //! Bool of planning node status
  std_msgs::Bool perform_planning_;
  //! Bool of having printed planner node status
  bool printed_deactivation_status_;

  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<docking::HolonomicPlannerNodeConfig> dr_srv_;
  //! Dynamic Reconfig Variables
  std::string dock_frame_;
  std::string robot_frame_;
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
  bool within_goal_dist_tolerance_;
  double goal_orientation_tolerance_;
  bool within_goal_orientation_tolerance_;
  std::string dock_pose_topic_;
  std::string dock_gazebo_pose_topic_;
  bool use_calculated_pose_;
  int max_steps_;
//  int frequency_;
//  ros::Rate frequencyRate_;
  double entrance_dist_;

  //! Control System Variables
  double kp_translate_, ki_translate_, kd_translate_;
  double kp_rotate_, ki_rotate_, kd_rotate_;
  double control_frequency_;
  geometry_msgs::Twist zeroTwist_;

  bool successY_, successX_, successTheta_;

  //! Odom
  tf2::Transform odom2BaseTF_, odom2EntranceTF_, deltaTF_;

  // Declaration and Definition
  void startDynamicReconfigureServer() {
  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server values can be overwritten.
    ROS_INFO_STREAM("startDynamicReconfigureServer: STARTING DYNAMIC RECONFIGURE SERVER");
    dynamic_reconfigure::Server<docking::HolonomicPlannerNodeConfig>::CallbackType cb;
    cb = boost::bind(&HolonomicPlannerNode::configCallback, this, _1, _2);
    dr_srv_.setCallback(cb);
  }

  //! Callback function for dynamic reconfigure server.
  void configCallback(docking::HolonomicPlannerNodeConfig &config, uint32_t level __attribute__((unused))) {
    robot_frame_ = config.robot_frame;
    omega_max_ = config.omega_max;
    kp_translate_ = config.kp_translate;
    ki_translate_ = config.ki_translate;
    kd_translate_ = config.kd_translate;
    kp_rotate_ = config.kp_rotate;
    ki_rotate_ = config.ki_rotate;
    kd_rotate_ = config.kd_rotate;
    control_frequency_= config.control_frequency;
//    lin_vel_min_ = config.v;
//    lin_vel_max_ = config.v;
    goal_orientation_tolerance_ = config.goal_orientation_tolerance;
    goal_dist_tolerance_ = config.goal_dist_tolerance;
    dock_pose_topic_ = config.dock_pose_topic;
    dock_gazebo_pose_topic_ = config.dock_gazebo_pose_topic;
    use_calculated_pose_ = config.use_calculated_pose;
  }

  void initParams() {
    std::cout << "initParams: INITIALIZING PARAMS FROM PARAM SERVER: " << std::endl;
    // Initialize node parameters from launch file or command line.
    nh_.param("kp_translate", kp_translate_, kp_translate_);
    nh_.param("ki_translate", ki_translate_, ki_translate_);
    nh_.param("kd_translate", kd_translate_, kd_translate_);
    nh_.param("kp_rotate", kp_rotate_, kp_rotate_);
    nh_.param("ki_rotate", ki_rotate_, ki_rotate_);
    nh_.param("kd_rotate", kd_rotate_, kd_rotate_);

  }

  void startPub() {
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    path_pub_ = nh_.advertise<nav_msgs::Path>("docking/path", 10);
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("docking/pose_array", 10);
    pose_executed_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("docking/pose_executed_array", 10);
    plan_pub_ = nh_.advertise<docking::Plan>("docking/plan", 10);
  }

//  void startPoseSub(std::string& dockPoseTopic)
  void startPoseSub()
  {
    if(use_calculated_pose_){
      ROS_WARN_STREAM("HolonomicPlannerNode: Subscribing to " << dock_pose_topic_);
      dockPoseSub_ = nh_.subscribe(dock_pose_topic_, 1, &HolonomicPlannerNode::dockPoseCallback, this);
      ROS_WARN_STREAM("HolonomicPlannerNode: Subscribed to " << dock_pose_topic_);
    } else {
      ROS_INFO_STREAM("HolonomicPlannerNode: Subscribing to " << dock_gazebo_pose_topic_);
      dockPoseSub_ = nh_.subscribe(dock_gazebo_pose_topic_, 1, &HolonomicPlannerNode::dockPoseCallback, this);
      ROS_INFO_STREAM("HolonomicPlannerNode: Subscribed to " << dock_gazebo_pose_topic_);
    }
  }

  void dockPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    // Only perform planning if activated
    if (perform_planning_.data == false) {
      if (!printed_deactivation_status_) {
        ROS_WARN_STREAM("dockPoseCallback: PLANNING NOT ACTIVATED");
        printed_deactivation_status_ = true;
      }
      return;
    }

    // ros::Time beginCallback = ros::Time::now();

    ROS_INFO_STREAM("DOCK POSE CALLBACK");

    geometry_msgs::TransformStamped odom2EntranceTFMsg = tfBuffer_.lookupTransform("odom", "entrance", ros::Time(0));
    tf2::convert(odom2EntranceTFMsg.transform, odom2EntranceTF_);
    ROS_INFO_STREAM("ODOM->ENTRANCE TRANSFORM" << transformString(odom2EntranceTF_));

    // ros::Duration total = ros::Time::now() - beginCallback;
    // ROS_WARN_STREAM("PLANNING TOOK " << total.toSec() << " secs");
  }

  void startActivationSub(){
    activationSub_ = nh_.subscribe("docking/perform_planning", 1, &HolonomicPlannerNode::activationCallback, this);
  }

  void startOdomSub(){
//    odomSub_ = nh_.subscribe("odom", 1, &HolonomicPlannerNode::odomCallback, this);
    odomSub_ = nh_.subscribe("odom", 1, &HolonomicPlannerNode::odomCallback, this,ros::TransportHints().tcpNoDelay());
    ros::Duration(0.3).sleep(); //sleep while we subscribe to the odom message
    ROS_INFO_STREAM("HolonomicPlannerNode: Subscribed to " << "odom");
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg){
    tf2::fromMsg(msg.get()->pose.pose, odom2BaseTF_);
    ROS_INFO_STREAM("ODOM->BASE TRANSFORM" << transformString(odom2BaseTF_));
  }

  void activationCallback(const std_msgs::BoolConstPtr &msg)
  {
      if(perform_planning_.data == msg->data){
        return;
      }
      perform_planning_ = *msg;
      ROS_INFO_STREAM("SETTING PLANNING ACTIVATION STATUS TO  " << perform_planning_);
      printed_deactivation_status_ = false;
    }

  void initGlobals(){
    ROS_INFO_STREAM("PlannerNode: Initializing Globals");
    successX_ = successY_ = successTheta_ = false;
    ros::Duration time_step_duration(time_step_);
    time_step_duration_ = time_step_duration;
//    ros::Rate frequencyRate(frequency_);
//    frequencyRate_ = frequencyRate;
    nh_.param("robot_frame", robot_frame_, robot_frame_);
    plan_.header.frame_id = plan_.path.header.frame_id = plan_.poseArray.header.frame_id = robot_frame_;
    zeroTwist_.linear.x = zeroTwist_.angular.z = 0.0;
    within_goal_dist_tolerance_ = within_goal_orientation_tolerance_ = false;
    ROS_INFO_STREAM("PlannerNode: Initialized Globals");
  }

  void printTranslationGains(){
    ROS_INFO("KP: %f, KI: %f, KD: %f ", kp_translate_, ki_translate_ , kd_translate_);
  }

  void printRotationGains(){
    ROS_INFO("KP: %f, KI: %f, KD: %f ", kp_rotate_, ki_rotate_ , kd_rotate_);
  }



  bool execute(){
    // Only perform planning if activated
    if (perform_planning_.data == false){
      if(!printed_deactivation_status_){
        ROS_WARN_STREAM("dockPoseCallback: PLANNING NOT ACTIVATED");
        cmd_vel_pub_.publish(zeroTwist_);
        printed_deactivation_status_ = true;
      }
      return false;
    }

    ROS_WARN_STREAM("EXECUTING Y TRANSLATION");
//    if(!successY_){
      if(translate("y", goal_dist_tolerance_))
        ROS_DEBUG_STREAM("Y TRANSLATION SUCCESSFUL");
      else{
        ROS_ERROR_STREAM("Y TRANSLATION FAILED");
        return false;
      }
//    }


    ros::Duration(1.0).sleep();

    ROS_WARN_STREAM("EXECUTING YAW ROTATION");
      if(rotate( goal_orientation_tolerance_))
        ROS_DEBUG_STREAM("YAW ROTATION SUCCESSFUL");
      else{
        ROS_ERROR_STREAM("YAW ROTATION FAILED");
        return false;
      }



    ROS_WARN_STREAM("EXECUTING X TRANSLATION");
//    if(!successX_){
      if(translate("x", goal_dist_tolerance_))
        ROS_DEBUG_STREAM("x TRANSLATION SUCCESSFUL");
      else{
        ROS_ERROR_STREAM("x TRANSLATION FAILED");
        return false;
      }
//    }

  }

  /// Check if entrance frame exists in TF tree
  /// \brief dockExists
  /// \return bool
  ///
  bool dockExists(){
    try {
      tfBuffer_.lookupTransform("odom", "entrance", ros::Time(0));
    }
    catch (tf::TransformException const &ex)  {
      ROS_ERROR("%s",ex.what());
      return false;
    }
    return true;
  }

  /// Translate in X or Y Axes
  /// \brief translate
  /// \param axis
  /// \param tolerance
  /// \return bool
  ///
  bool translate(std::string axis, double tolerance)
  {
    ros::Rate control_rate(control_frequency_); // freq we want to control the position at
    double drive_effort = 0.0;
    double initial_error = 0.0;
    double position_error = 0.0;
    double position_error_diff = 0.0;
    double last_position_error = 0.0;
    double position_error_sum = 0.0;
    geometry_msgs::Twist twist = zeroTwist_;
    ros::Time start_time = ros::Time::now();
    ros::Time last_update_time = ros::Time::now();
    double dT = 0.0;
    if(!axis.compare("x"))
      initial_error = position_error = odom2BaseTF_.inverseTimes(odom2EntranceTF_).getOrigin().getX();
    if(!axis.compare("y"))
      initial_error = position_error = odom2BaseTF_.inverseTimes(odom2EntranceTF_).getOrigin().getY();
    ROS_INFO_STREAM("Initial Error of " << initial_error);
    ROS_INFO_STREAM("Goal Dist Tolerance " << tolerance);
    bool stop = false;

    while(!stop && ros::ok()){
      if(!dockExists()){
        cmd_vel_pub_.publish(zeroTwist_);
        ROS_WARN_STREAM("DOCK NOT FOUND");
        ROS_INFO_STREAM("Waiting 0.1 seconds for dock frame to exist");
        ros::Duration(0.1).sleep();
        if(!dockExists()){
          ROS_WARN_STREAM("DOCK STILL NOT FOUND. TRANSLATION FAILED");
          return false;
        }
      }
      printTranslationGains();

      if(fabs(position_error) < tolerance)
        stop = true;

      dT = (ros::Time::now()-last_update_time).toSec();
      last_update_time =  ros::Time::now();
      if(!axis.compare("x"))
        position_error = odom2BaseTF_.inverseTimes(odom2EntranceTF_).getOrigin().getX();
      if(!axis.compare("y"))
        position_error = odom2BaseTF_.inverseTimes(odom2EntranceTF_).getOrigin().getY();
      ROS_INFO_STREAM("Current Position Error of " << position_error);
      position_error_diff = position_error - last_position_error;
      position_error_sum += position_error * dT;
      //store our last position error
      last_position_error = position_error;
      //Limit the integrator
      if (fabs(position_error_sum) > 1.0)
        position_error_sum = copysign(1.0, position_error_sum);

      //PID control law...
      drive_effort = position_error*kp_translate_ + position_error_diff*kd_translate_ + position_error_sum*ki_translate_;
      ROS_INFO("PP: %f, PI: %f, PD: %f ", position_error, position_error_sum, position_error_diff);
      ROS_INFO("PP: %f, PI: %f, PD: %f ", position_error*kp_translate_, position_error_sum*ki_translate_, position_error_diff*kd_translate_);


      if(drive_effort < 0.07 && drive_effort < 0)
        drive_effort = -0.07;
      if(drive_effort < 0.07 && drive_effort > 0)
        drive_effort = 0.07;

      ROS_INFO_STREAM("Drive Effort " << drive_effort);

      if(!axis.compare("x")){
        twist.linear.x = drive_effort;
      }  
      else if(!axis.compare("y")){
        twist.linear.y = drive_effort;
      }

      cmd_vel_pub_.publish(twist);

      ros::spinOnce();
      control_rate.sleep();

    }

    cmd_vel_pub_.publish(zeroTwist_);
    if(!axis.compare("x"))
      successX_ = true;
    if(!axis.compare("y"))
      successY_ = true;
    return true;

  }

  /// Rotate about Z Axis (Yaw)
  /// \brief rotate
  /// \param tolerance
  /// \return
  ///
  bool rotate(double tolerance)
  {
    ros::Rate control_rate(control_frequency_); // freq we want to control the position at
    double rotation_effort = 0.0;
    double initial_error = 0.0;
    double rotation_error = 0.0;
    double rotation_error_diff = 0.0;
    double last_rotation_error = 0.0;
    double rotation_error_sum = 0.0;
    geometry_msgs::Twist twist = zeroTwist_;
    ros::Time start_time = ros::Time::now();
    ros::Time last_update_time = ros::Time::now();
    double dT = 0.0;
    initial_error = rotation_error = tf2::getYaw(odom2BaseTF_.inverseTimes(odom2EntranceTF_).getRotation());
    ROS_INFO_STREAM("Initial Error of " << initial_error);
    ROS_INFO_STREAM("Goal Rotation Tolerance " << tolerance);
    bool stop = false;

    while(!stop && ros::ok()){
      if(!dockExists()){
        cmd_vel_pub_.publish(zeroTwist_);
        ROS_WARN_STREAM("DOCK NOT FOUND");
        ROS_INFO_STREAM("Waiting 0.1 seconds for dock frame to exist");
        ros::Duration(0.1).sleep();
        if(!dockExists()){
          ROS_WARN_STREAM("DOCK STILL NOT FOUND. ROTATION FAILED");
          return false;
        }
      }
      printRotationGains();

      if(fabs(rotation_error) < tolerance)
        stop = true;

      dT = (ros::Time::now()-last_update_time).toSec();
      last_update_time =  ros::Time::now();
      rotation_error = tf2::getYaw(odom2BaseTF_.inverseTimes(odom2EntranceTF_).getRotation());
      ROS_INFO_STREAM("Current ROTATION Error of " << rotation_error);
      rotation_error_diff = rotation_error - last_rotation_error;
      rotation_error_sum += rotation_error * dT;
      //store our last rotation error
      last_rotation_error = rotation_error;
      //Limit the integrator
      if (fabs(rotation_error_sum) > 1.0)
        rotation_error_sum = copysign(1.0, rotation_error_sum);

      //PID control law...
      rotation_effort = rotation_error*kp_translate_ + rotation_error_diff*kd_translate_ + rotation_error_sum*ki_translate_;
      ROS_INFO("PP: %f, PI: %f, PD: %f ", rotation_error, rotation_error_sum, rotation_error_diff);
      ROS_INFO("PP: %f, PI: %f, PD: %f ", rotation_error*kp_translate_, rotation_error_sum*ki_translate_, rotation_error_diff*kd_translate_);


      if(rotation_effort < 0.6 && rotation_effort < 0)
        rotation_effort = -0.6;
      if(rotation_effort < 0.6 && rotation_effort > 0)
        rotation_effort = 0.6;

      ROS_INFO_STREAM("Rotation Effort " << rotation_effort);

      twist.angular.z = rotation_effort;


      cmd_vel_pub_.publish(twist);

      ros::spinOnce();
      control_rate.sleep();

    }

    cmd_vel_pub_.publish(zeroTwist_);

    return true;

  }
};

#endif // HOLONOMICPLANNERNODE_H
