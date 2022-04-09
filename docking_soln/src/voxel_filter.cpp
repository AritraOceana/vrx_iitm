#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

ros::Publisher pub;

/*
 *
 * Applies a voxel filter to the input point cloud data.
 * Here, only the points within the voxel of (0.1, 0.1, 0.1) are replaced by their centroids
 */
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
    pub.publish(output);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "voxel_filter");
    ros::NodeHandle nh;

    // Create ROS Subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/passthrough_filtered_cloud", 1, cloud_cb);

    // Create a ROS Publisher Object
    pub = nh.advertise<sensor_msgs::PointCloud2>("/voxel_filtered_cloud", 1);

    ros::spin();
}