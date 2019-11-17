// Include the ROS library
#include <ros/ros.h>
#include <docking/SegmentLine.h>
//#include <docking/impl/SegmentLine.hpp>
//#include "SegmentLine.cpp"
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

// ROS Publisher
ros::Publisher pub;
int max_iter;
float dist_thresh;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
//void cloud_cb(const pcl::PointCloud<PointT>::ConstPtr &msg)
{
    // Container for original & filtered data
    /*
    * CONVERT POINTCLOUD ROS->PCL
    */
    pcl::PointCloud<PointT> cloud;
    pcl::fromROSMsg (*msg, cloud);
    pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT> (cloud));
    pcl::PointCloud<PointT>::Ptr cloud_voxel_filtered (new pcl::PointCloud<PointT> ());

    SegmentLine<PointT> *segmentLine = new SegmentLine<PointT>();
    segmentLine->setInputCloud(cloud_ptr);
    ROS_INFO_STREAM("Distance Threshold: " << dist_thresh);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> lines = segmentLine->RansacLine(cloud_ptr, max_iter, dist_thresh);
//    std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> line =
//    segmentLine->RansacLine();

//    std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> line = segmentLine->RansacLine(*msg, 1000, 0.01f);

    /* ========================================
     * CONVERT POINTCLOUD PCL->ROS
     * PUBLISH CLOUD
     * ========================================*/
    sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*lines.at(1), *pc2_cloud);
//    pc2_cloud->header.frame_id=world_frame;
//    pc2_cloud->header.stamp=ros::Time::now();
    pub.publish(pc2_cloud);
}

int main(int argc, char **argv)
{
    // Initialize the ROS Node "line_detection"
    ros::init(argc, argv, "line_detection");
    ros::NodeHandle nh;

    max_iter = nh.param<int>("max_iterations", 50);
    dist_thresh = nh.param<float>("distance_threshold", 0.01);

    // Print "Hello" message with node name to the terminal and ROS log file
    ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

    // Create a ROS Subscriber to INPUT_CLOUD with a queue_size of 1 and a callback function to cloud_cb
    ros::Subscriber sub = nh.subscribe(INPUT_CLOUD, 1, cloud_cb);

    // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
//    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);
    // create a templated publisher
    pub = nh.advertise<pcl::PointCloud<PointT>>(PUBLISH_TOPIC, 1);

    // Spin
    ros::spin();

    // Success
    return 0;
}
