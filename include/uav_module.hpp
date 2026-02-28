#pragma once

#include <rclcpp/rclcpp.hpp>
#include <prometheus_msgs/msg/uav_state.hpp>
#include <memory>

// 前向声明：告诉编译器 DataManager 是个类，但先不用包含它的细节
// 前向声明 DataManager，避免头文件循环包含
class DataManager;

class UavModule : public rclcpp::Node  {
public:
    UavModule(std::shared_ptr<DataManager> data_manager);
    // ~UavModule();

private:
    // 回调函数：当收到无人机状态消息时自动触发
    void uav_state_callback(const prometheus_msgs::msg::UAVState::SharedPtr msg);

    // ROS 订阅者对象
    rclcpp::Subscription<prometheus_msgs::msg::UAVState>::SharedPtr subscription_;

    std::shared_ptr<DataManager> data_manager_;
};