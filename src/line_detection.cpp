/*
*/

// Include the ROS library
#include <ros/ros.h>
#include "docking/SegmentLine.h"
// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

// Topics
static const std::string IMAGE_TOPIC = "/cloud_pc2";
static const std::string PUBLISH_TOPIC = "/pcl/points";

// ROS Publisher
ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud_pc2 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_pc2);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud_pc2);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.001, 0.001, 0.001);
    sor.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcxyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_pc2, *cloud_pcxyz);

    // SegmentLine<pcl::PointXYZ> *segmentLine = new SegmentLine<pcl::PointXYZ>;
    // std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> line = segmentLine.RansacLine(cloud_pcxyz, 1000, 0.01);

    // Publish the data
    pub.publish(output);
}

int main(int argc, char **argv)
{
    // Initialize the ROS Node "roscpp_pcl_example"
    ros::init(argc, argv, "roscpp_pcl_example");
    ros::NodeHandle nh;

    // Print "Hello" message with node name to the terminal and ROS log file
    ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

    // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
    ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

    // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

    // Spin
    ros::spin();

    // Success
    return 0;
}
