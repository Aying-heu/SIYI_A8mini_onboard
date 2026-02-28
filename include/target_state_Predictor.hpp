#pragma once
#include <Eigen/Dense>
#include <deque>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <iostream>
#include "data_types.hpp"
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include "prometheus_msgs/msg/target_array.hpp"

// 定义模型会话结构
struct ModelSession_predictor {
    Ort::Session session{nullptr};
    int batch_size = 0;
    int n_vars = 0;   // 特征数量 (例如 9: x,y,z, vx,vy,vz, ax,ay,az)
    int seq_len = 0;  // 输入长度 (450)
    int pred_len = 0; // 输出长度 (75)
    
    // 必须保存 string 实例，因为 input_names 存的是 char* 指针
    std::vector<std::string> input_names_str;
    std::vector<std::string> output_names_str;
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
};



// 前向声明 DataManager，避免头文件循环引用
class DataManager;

class TargetStatePredictor {
public:
    // 构造函数传入 DataManager 指针
    TargetStatePredictor(std::shared_ptr<DataManager> data_manager);
    ~TargetStatePredictor();

    // 启动预测器线程
    bool start();
    // 停止预测器线程
    void stop();

    // Offline: 外部手动驱动 (不要和 start() 同时使用)
    // 在离线读取 Log 的循环中，每次更新完时间后调用此函数
    void tick(uint64_t current_time_ns);

private:
    // 线程主循环
    void run();

    // 辅助函数，用于执行单次采样并存入Buffer
    void process_single_sample(uint64_t timestamp_ns);

    // 加载模型辅助函数
    void load_model(const std::string& model_path);

    // 核心预测算法：基于 history 预测未来的 future_count 帧
    std::vector<TargetState> predict_future(const std::deque<TargetState>& history);

    // 保存日志
    void save_to_binary(const std::deque<TargetState>& history, const std::vector<TargetState>& future);

    // 辅助：TargetState 转 float vector (根据你的模型特征定义)
    void state_to_features(const TargetState& state, std::vector<float>& features);
    // 辅助：float vector 转 TargetState
    TargetState features_to_state(const float* features, uint64_t base_time, int step_index);
    prometheus_msgs::msg::Target state_to_ros_msg(
                                    const TargetState& state, 
                                    uint64_t base_time_ns, 
                                    int step_idx, 
                                    uint64_t dt_ns);


private:
    std::shared_ptr<DataManager> data_manager_;
    rclcpp::Publisher<prometheus_msgs::msg::TargetArray>::SharedPtr target_array_pub_;

    
    // 状态控制
    std::atomic<bool> is_running_{false};
    std::thread predictor_thread_;

    // 数据缓存
    std::deque<TargetState> history_buffer_;
    // 注意：这些值会被模型加载后的实际维度覆盖，这里是默认值
    size_t max_history_size_ = 450;
    int prediction_count_ = 75;

    std::mutex buffer_mtx_;

    // ONNX Runtime 环境 (必须在 Session 之前初始化)
    Ort::Env ort_env_{nullptr};
    ModelSession_predictor predictor_model_;

    // 时间控制 (纳秒)
    // 15Hz => 1000000000 / 15 ≈ 66666666 ns
    // 10Hz => 1000000000 / 10 = 100000000 ns
    const uint64_t sample_interval_ns_ = 100000000; 
    const uint64_t predict_interval_ns_ = 500000000;      // 0.5s
    const uint64_t sync_interval_ns_    = 5000000000;     // 5.0s

    uint64_t next_sample_time_ns_ = 0; // 下一次应该采样的绝对时间戳
    uint64_t last_predict_time_ns_ = 0;
    uint64_t last_sync_time_ns_ = 0;
};