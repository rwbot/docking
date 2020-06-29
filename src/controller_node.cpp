#include <docking/Headers.h>
#include <docking/ControllerNode.h>
#include <ros/ros.h>

// Topics
//static const std::string DOCK_FRAME = "dock_truth";

//static const std::string PUBLISH_TOPIC = "/dockLines";
//geometry_msgs::PoseStamped targetPose;

int main(int argc, char **argv)
{
  ROS_INFO_STREAM("INITIALIZING CONTROLLER NODE");
  // Initialize the ROS Node "line_detection_node"
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh;
  // ros::NodeHandle nh("");
  //    ros::NodeHandle nh("~");

  ControllerNode *controllerNode = new ControllerNode(nh);

  ros::Rate rate(15);

  // Spin
  while(ros::ok())
  {
//    ROS_INFO_STREAM("CONTROLLER SPINNING");
    controllerNode->publishActivation();
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO_STREAM("EXITING NODE");
  // Success
  return 0;
}
