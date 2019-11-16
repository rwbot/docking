#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // pcl::fromROSMsg(*input, cloud);

    // pcl::ModelCoefficients coefficients;
    // pcl::PointIndices inliers;
    // // Create the segmentation object
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // // Optional
    // seg.setOptimizeCoefficients(true);
    // // Mandatory
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setDistanceThreshold(0.01);

    // seg.setInputCloud(cloud.makeShared());
    // seg.segment(inliers, coefficients);

    // // Publish the model coefficients
    // pcl_msgs::ModelCoefficients ros_coefficients;
    // pcl_conversions::fromPCL(coefficients, ros_coefficients);
    // pub.publish(ros_coefficients);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("input", 1, cloud_cb);

    // Create a ROS publisher for the output model coefficients
    // pub = nh.advertise<pcl_msgs::ModelCoefficients>("output", 1);

    // Spin
    ros::spin();
}
