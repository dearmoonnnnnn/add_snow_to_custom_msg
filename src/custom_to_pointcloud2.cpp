#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub_cloud;

void customMsgCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    cloud->header.frame_id = msg->header.frame_id;

    for (const auto& p : msg->points) {
        pcl::PointXYZI pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = p.reflectivity;
        cloud->points.push_back(pt);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = msg->header;

    output.header.frame_id = "map";

    pub_cloud.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_to_pointcloud2");
    ros::NodeHandle nh;

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/livox/pointcloud2_with_snow", 1000);

    ros::Subscriber sub = nh.subscribe("/livox/lidar_with_snow", 10, customMsgCallback);

    ros::spin();
    return 0;
}
