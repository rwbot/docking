// Include the ROS library
#include <ros/ros.h>
#include <docking/SegmentLineNode.h>
//#include <docking/impl/SegmentLineNode.hpp>
//#include "SegmentLineNode.cpp"
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
//#include <pcl/sample_consensus/sac_model_line.h>


// Topics
static const std::string INPUT_CLOUD = "/cloud";
static const std::string PUBLISH_TOPIC = "/dockLines";

typedef pcl::PointXYZI PointT;


int main(int argc, char **argv)
{
    // Initialize the ROS Node "line_detection_node"
    ros::init(argc, argv, "line_detection_node");
    ros::NodeHandle nh;

    //docking::SegmentLineNode<PointT> *segmentLineNode = new docking::SegmentLineNode<PointT>(nh);

    // Spin
    ros::spin();

    // Success
    return 0;
}
