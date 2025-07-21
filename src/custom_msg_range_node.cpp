#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <limits>
#include <iostream>
#include <iomanip>

float min_x = std::numeric_limits<float>::max();
float max_x = std::numeric_limits<float>::lowest();
float min_y = min_x, max_y = max_x;
float min_z = min_x, max_z = max_x;
float max_offset_time = std::numeric_limits<float>::lowest();

ros::Time last_msg_time;
double timeout = 5.0;   // 5 秒没消息视为结束

void cloudCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    ROS_INFO("cloudCallback is running...");
    last_msg_time = ros::Time::now();
    if (msg->point_num == 0) {
        ROS_WARN("收到的点云为空");
        return;
    }

    for (const auto& point : msg->points) {
        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.y > max_y) max_y = point.y;
        if (point.z < min_z) min_z = point.z;
        if (point.z > max_z) max_z = point.z;
        if (point.offset_time > max_offset_time) max_offset_time = point.offset_time;  
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_msg_range_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/livox/lidar", 10, cloudCallback);

    last_msg_time = ros::Time::now();
    ros::Rate rate(10);  // 10Hz

    while (ros::ok()) {
        ros::spinOnce();

        if ((ros::Time::now() - last_msg_time).toSec() > timeout) {
            ROS_INFO_STREAM("点云接收完毕，坐标范围如下：");
            ROS_INFO_STREAM("X: [" << min_x << ", " << max_x << "]");
            ROS_INFO_STREAM("Y: [" << min_y << ", " << max_y << "]");
            ROS_INFO_STREAM("Z: [" << min_z << ", " << max_z << "]");
            ROS_INFO_STREAM("offset_time: [0," << std::fixed << std::setprecision(0) << max_offset_time << "]");            
            break;
        }

        rate.sleep();
    }

    ros::shutdown();
    return 0;
}
