#include <docking/Headers.h>
#include <docking/PlannerNode.h>
#include <ros/ros.h>


// Topics
//static const std::string DOCK_FRAME = "dock_truth";

//static const std::string PUBLISH_TOPIC = "/dockLines";
//geometry_msgs::PoseStamped targetPose;

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("INITIALIZING PLANNER NODE");
    // Initialize the ROS Node "line_detection_node"
    ros::init(argc, argv, "planner");
     ros::NodeHandle nh;
    // ros::NodeHandle nh("");
//    ros::NodeHandle nh("~");

    PlannerNode *plannerNode = new PlannerNode(nh);

    ros::Rate rate(1);

    // Spin
   while(ros::ok())
   {
    //   ros::spin();
    ros::spinOnce();
    rate.sleep();
   }



    // Success
    return 0;
}
