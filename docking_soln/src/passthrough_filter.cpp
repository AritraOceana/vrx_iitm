#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

ros::Publisher pub;

/*
*
* Applies a passthrough filter to the input point cloud data.
* Here, only the points with x > 0 and x < 20 are retained, i.e. the points other than that will be rejected
*
*/
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // Container for output data
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Process the data here
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 20);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(-2, 3);
    // pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);

    pcl::toROSMsg(*cloud_filtered, output);

    // Publish the output point cloud of type (ros sensor_msgs)
    ROS_INFO_STREAM("Filtered Output can be seen on:\t/passthrough_filtered_cloud");
    pub.publish(output);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "passthrough_filter");
    ros::NodeHandle nh;

    // Create ROS Subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/wamv/sensors/lidars/lidar_wamv/points", 1, cloud_cb);

    // Create a ROS Publisher Object
    pub = nh.advertise<sensor_msgs::PointCloud2>("/passthrough_filtered_cloud", 1);

    ros::spin();
}