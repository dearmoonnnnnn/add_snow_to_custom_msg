#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <random>
#include <rosbag/bag.h>

ros::Publisher snow_pub;

rosbag::Bag bag;

// 静态随机数生成器
static std::random_device rd;
static std::mt19937 gen(rd());

void addSnowPoints(livox_ros_driver::CustomMsg &msg, int snow_count) {
    // 根据点云密度动态调整雪点数量
    int adjusted_count = snow_count;
    if(msg.point_num > 0) {
        float density = msg.point_num / (40.0f * 40.0f); // 假设FOV为40x40米
        adjusted_count = static_cast<int>(snow_count * density / 10.0f);
    }
    
    // 根据Horizon激光雷达参数调整分布(81.7°H x 25.1°V FOV, 260m范围)
    const float max_dist = 100.0f; // 实际使用100米范围
    const float h_range = tan(0.713f) * max_dist; // 81.7°/2=0.713rad
    const float v_range = tan(0.219f) * max_dist; // 25.1°/2=0.219rad
    
    std::normal_distribution<float> dist_x(0.0, h_range/2); // 水平方向高斯分布
    std::normal_distribution<float> dist_y(0.0, h_range/2);
    std::normal_distribution<float> dist_z(0.0, v_range/2); // 垂直方向高斯分布
    std::normal_distribution<float> dist_dist(0.0, max_dist/3); // 距离分布
    std::uniform_int_distribution<int> dist_line(0, 7);
    std::normal_distribution<float> dist_reflect(70, 15); // 雪点反射率集中在70左右

    for (int i = 0; i < adjusted_count; ++i) {
        livox_ros_driver::CustomPoint snow_point;
        float distance = fabs(dist_dist(gen));
        float angle_h = dist_x(gen)/h_range * 0.713f; // 水平角度(rad)
        float angle_v = dist_z(gen)/v_range * 0.219f; // 垂直角度(rad)
        
        snow_point.x = distance * tan(angle_h);
        snow_point.y = distance * tan(angle_h); // 简化模型
        snow_point.z = distance * tan(angle_v);
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

    // 添加雪点，基础数量100但会根据密度自动调整
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
