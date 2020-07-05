#include <docking/Headers.h>
#include <docking/HolonomicPlannerNode.h>
#include <ros/ros.h>


// Topics
//static const std::string DOCK_FRAME = "dock_truth";

//static const std::string PUBLISH_TOPIC = "/dockLines";
//geometry_msgs::PoseStamped targetPose;

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("INITIALIZING HOLONOMIC PLANNER NODE");
    // Initialize the ROS Node ""
    ros::init(argc, argv, "holonomic_planner");
     ros::NodeHandle nh; 
    // ros::NodeHandle nh("");
//    ros::NodeHandle nh("~");

    HolonomicPlannerNode *holonomicPlannerNode = new HolonomicPlannerNode(nh);

    ros::Rate rate(1);

    // Spin
   while(ros::ok())
   {
    //   ros::spin();
    holonomicPlannerNode->execute();
    ros::spinOnce();
    rate.sleep();
   }



    // Success
    return 0;
}
