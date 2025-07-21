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
    
    // 使用用户指定的XYZ范围生成雪点(高斯分布)
    const float x_min = 0.0f, x_max = 143.574f;
    const float y_min = -55.07f, y_max = 75.186f;
    const float z_min = -1.765f, z_max = 20.606f;
    
    // 计算各轴中心和标准差(范围/4)
    const float x_center = (x_min + x_max) / 2.0f;
    const float y_center = (y_min + y_max) / 2.0f;
    const float z_center = (z_min + z_max) / 2.0f;
    const float x_stddev = (x_max - x_min) / 4.0f;
    const float y_stddev = (y_max - y_min) / 4.0f;
    const float z_stddev = (z_max - z_min) / 4.0f;
    
    std::normal_distribution<float> dist_x(x_center, x_stddev);
    std::normal_distribution<float> dist_y(y_center, y_stddev);
    std::normal_distribution<float> dist_z(z_center, z_stddev);
    std::uniform_int_distribution<int> dist_line(0, 7);
    std::normal_distribution<float> dist_reflect(70, 15); // 雪点反射率集中在70左右

    for (int i = 0; i < adjusted_count; ++i) {
        livox_ros_driver::CustomPoint snow_point;
        
        // 生成点并确保在范围内
        do {
            snow_point.x = dist_x(gen);
        } while (snow_point.x < x_min || snow_point.x > x_max);
        
        do {
            snow_point.y = dist_y(gen);
        } while (snow_point.y < y_min || snow_point.y > y_max);
        
        do {
            snow_point.z = dist_z(gen);
        } while (snow_point.z < z_min || snow_point.z > z_max);

        // offset_time
        uint32_t max_offset_time = 100142368;  // 100142368
        static std::default_random_engine gen(std::random_device{}());
        std::uniform_int_distribution<uint32_t> dist_offset(0, max_offset_time);
        snow_point.offset_time = dist_offset(gen);


        int refl = static_cast<int>(dist_reflect(gen));
        snow_point.reflectivity = std::max(0, std::min(255, refl)); // 裁减边界

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
    ROS_INFO("output path: %s", outputFile.c_str());
    bag.open(outputFile, rosbag::bagmode::Write);

    ros::Subscriber sub = nh.subscribe<livox_ros_driver::CustomMsg>(
        "/livox/lidar", 4000, cloudCallback);

    snow_pub = nh.advertise<livox_ros_driver::CustomMsg>(
        "/livox/lidar_with_snow", 4000);

    ros::spin();

    bag.close();
    return 0;
}
