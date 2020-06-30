// Include the ROS library
#include <ros/ros.h>
#include <docking/DetectionNode.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>
// Include pcl
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>

int main(int argc, char **argv)
{
    // Initialize the ROS Node "detection_node"
    ros::init(argc, argv, "detection_node");
    ros::NodeHandle nh;
    // ros::NodeHandle nh("");
    // ros::NodeHandle nh("~");

    DetectionNode *detectionNode = new DetectionNode(nh);

    ros::Rate rate(5);
    // Spin
    while(ros::ok()){
      ros::spinOnce();
      rate.sleep();
    }

    // Success
    return 0;
}
