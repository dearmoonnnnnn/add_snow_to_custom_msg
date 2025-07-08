#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <random>
#include <rosbag/bag.h>

ros::Publisher snow_pub;

rosbag::Bag bag;

void addSnowPoints(livox_ros_driver::CustomMsg &msg, int snow_count) {
    std::default_random_engine gen;
    std::uniform_real_distribution<float> dist_x(-20.0, 20.0);
    std::uniform_real_distribution<float> dist_y(-20.0, 20.0);
    std::uniform_real_distribution<float> dist_z(0.0, 3.0);
    std::uniform_int_distribution<int> dist_line(0, 7);
    std::uniform_int_distribution<int> dist_reflect(50, 100);

    for (int i = 0; i < snow_count; ++i) {
        livox_ros_driver::CustomPoint snow_point;
        snow_point.x = dist_x(gen);
        snow_point.y = dist_y(gen);
        snow_point.z = dist_z(gen);
        snow_point.offset_time = 0.0;          // 可根据需要设置时间偏移
        snow_point.reflectivity = dist_reflect(gen);
        snow_point.line = dist_line(gen);
        snow_point.tag = msg.points[0].tag;    // tag 与原来的点保持一致

        msg.points.push_back(snow_point);
        msg.point_num++;
    }
}

void cloudCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg_in) {
    livox_ros_driver::CustomMsg msg_out = *msg_in;  // 拷贝一份

    // 添加雪点
    addSnowPoints(msg_out, 100);

    // 发布
    snow_pub.publish(msg_out);

    // 写入 bag 文件
    bag.write("/livox/lidar", msg_in->header.stamp, msg_out);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_snow_point_injector");
    ros::NodeHandle nh("~");

    std::string outputFile = nh.param<std::string>("outputFile", "/test.bag");
    bag.open(outputFile, rosbag::bagmode::Write);

    ros::Subscriber sub = nh.subscribe<livox_ros_driver::CustomMsg>(
        "/livox/lidar", 4000, cloudCallback);

    snow_pub = nh.advertise<livox_ros_driver::CustomMsg>(
        "/livox/lidar_with_snow", 4000);

    ros::spin();

    bag.close();
    return 0;
}
