// include/my_uav_vision/data_manager.hpp
#pragma once
#include "data_types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <mutex>
#include <deque>
#include <condition_variable>
#include <prometheus_msgs/msg/target.hpp>

#include <filesystem>
#include "target_state_Filter.hpp"
namespace fs = std::filesystem;

class DataManager {
public:
    DataManager(rclcpp::Node::SharedPtr node,const std::string& root_path_);
    // --- 1. 时间接口 (Public) ---
    uint64_t now_ns() {
        // rclcpp:Time     clock_->now()
        // clock_->now().nanoseconds()      long long = uint
        // nanoseconds  =  seconds + nanosec     * k?
        return clock_->now().nanoseconds();
    }
    rclcpp::Node::SharedPtr node() const { return node_; }
    std::shared_ptr<DataConfig> get_config() const { return std::make_shared<DataConfig>(data_config_); }

    // --- 2. 线程安全推入 ---
    void pushUavState(const UAV_Row& state);
    void pushGimbalState(const Gimbal_Row& state);
    void pushRawImage(const cv::Mat& img, uint64_t ts);

    void pushDetectedImage(const Img_Row& row);
    void pushTargetMsg(prometheus_msgs::msg::Target target_msg_ori,prometheus_msgs::msg::Target target_msg_filtered);
    void pushSample15hzTargetstate(TargetState target_state_);

    // --- 3. 线程安全查询 ---
    // 增加了一个 wait 参数，如果没有图，视觉线程会在这里“睡觉”等待，不占CPU
    bool waitForImage(Img_Row& out_row);
    bool getUavStateAt(uint64_t ts, UAV_Row& out_state);
    bool getLatestUavState(UAV_Row& out_state);
    bool getGimbalStateAt(uint64_t ts, Gimbal_Row& out_state);
    bool getLatestGimbalState(Gimbal_Row& out_state);
    bool getLatestTargetMsg(prometheus_msgs::msg::Target& target_msg);
    void updateTargetStateFilterParameters(uint64_t t_ns, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat_meas);
    TargetState get_target_filtered_state(uint64_t query_time_ns);

    // --- 3.1 目标状态预测接口 ---
    // 获取当前时刻的目标预测状态（15Hz调用）
    bool getPredictedTargetState(prometheus_msgs::msg::Target& predicted_target);

    // --- 4. 存储接口 (给 DataLogger 用) ---
    // 一次性把所有“用过”的数据取走用于写CSV，并清空内部容器
    void popUsedData(
            std::vector<UAV_Row>& uav_list, 
            std::vector<Gimbal_Row>& gimbal_list,
            std::vector<Img_Row>& img_list,
            std::vector<prometheus_msgs::msg::Target>& target_list_ori,
            std::vector<prometheus_msgs::msg::Target>& target_list_filtered,
            std::vector<TargetState>& target_list_15hz
            );

private:

    int MAX_UAV_SIZE = 1000;
    int MAX_GIMBAL_SIZE = 1000;
    int MAX_TARGET_SIZE = 450;
    int MAX_IMG_SIZE = 10;

    // 1. 图像队列 (只存最新的几帧)
    std::deque<Img_Row> raw_img_deque_;
    std::mutex img_mtx_;
    std::condition_variable img_cv_;

    std::map<TimestampNS, UAV_Row> uav_history_;
    std::mutex uav_mtx_;
    
    std::map<TimestampNS, Gimbal_Row> gimbal_history_;
    std::mutex gimbal_mtx_;

    // 待保存数据（已用过）
    std::vector<UAV_Row> used_uav_history_;
    std::mutex used_uav_mtx_;
    std::vector<Gimbal_Row> used_gimbal_history_;
    std::mutex used_gimbal_mtx_;
    std::vector<Img_Row> used_img_vector_;
    std::mutex used_img_mtx_;
    std::vector<prometheus_msgs::msg::Target> detected_target_msg_;
    std::vector<prometheus_msgs::msg::Target> filtered_target_msg_;
    std::mutex target_msg__mtx_;
    std::vector<TargetState> sample_15hz_target_state_;
    std::mutex sample_15hz_target_mtx_;


    rclcpp::Node::SharedPtr node_;
    rclcpp::Clock::SharedPtr clock_; // 核心时钟源  
    DataConfig data_config_;
    
    // 目标跟踪器（卡尔曼滤波）
    TargetStateFilter target_state_filter_;
    std::mutex target_state_filter_mtx_;
};