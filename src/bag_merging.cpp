#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "bag_merger");
    ros::NodeHandle nh("~");

    // 读取参数
    std::string bag1_path, bag2_path, output_bag_path;
    if (!nh.getParam("bag1", bag1_path)) {
        ROS_ERROR("Failed to get 'bag1' parameter");
        return -1;
    }
    if (!nh.getParam("bag2", bag2_path)) {
        ROS_ERROR("Failed to get 'bag2' parameter");
        return -1;
    }
    if (!nh.getParam("output_bag", output_bag_path)) {
        ROS_ERROR("Failed to get 'output_bag' parameter");
        return -1;
    }

    ROS_INFO("[Bag Merger] Starting bag merge process");

    try {
        // 打开输入bag文件和输出bag文件
        rosbag::Bag bag1(bag1_path, rosbag::bagmode::Read);
        rosbag::Bag bag2(bag2_path, rosbag::bagmode::Read);
        rosbag::Bag output_bag(output_bag_path, rosbag::bagmode::Write);

        // 定义要处理的话题
        std::vector<std::string> topics_bag1 = {"/livox/imu", "/color/compressed"};
        std::vector<std::string> topics_bag2 = {"/livox/lidar"};

        // 创建视图
        rosbag::View view1(bag1, rosbag::TopicQuery(topics_bag1));
        rosbag::View view2(bag2, rosbag::TopicQuery(topics_bag2));
        
        // 计算总消息数
        size_t total_msgs = view1.size() + view2.size();

        // 开始处理第一个bag的消息
        size_t count1 = 0;
        for (const rosbag::MessageInstance& msg : view1) {
            output_bag.write(msg.getTopic(), msg.getTime(), msg);
            count1++;
        }
        ROS_INFO_STREAM("  Processed " << count1 << " messages from Bag1");

        // 开始处理第二个bag的消息
        size_t count2 = 0;
        for (const rosbag::MessageInstance& msg : view2) {
            output_bag.write("/livox/lidar", msg.getTime(), msg);
            count2++;
        }
        ROS_INFO_STREAM("  Processed " << count2 << " messages from Bag2");

        // 打印完成信息
        ROS_INFO_STREAM("[Bag Merger] Merge completed successfully!");
        ROS_INFO_STREAM("  Total messages merged: " << (count1 + count2));
        
        if (count1 + count2 != total_msgs) {
            ROS_WARN("  Warning: Processed count (%zu) doesn't match expected count (%zu)", 
                     count1 + count2, total_msgs);
        } else {
            ROS_INFO("  Message counts verified");
        }

        // 关闭所有bag文件
        bag1.close();
        bag2.close();
        output_bag.close();
        
        ROS_INFO("Output bag created: %s", output_bag_path.c_str());

    } catch (const rosbag::BagException& e) {
        ROS_ERROR_STREAM("Bag operation failed: " << e.what());
        return -1;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Unexpected error: " << e.what());
        return -1;
    }

    return 0;
}