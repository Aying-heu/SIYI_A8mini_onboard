#pragma once

#include <Eigen/Dense>
#include <mutex>
#include <iostream>
#include "data_types.hpp" 

// 追踪器输出状态结构体
struct TrackerState {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    TimestampNS ts;
    bool is_valid = false;
};

class TargetTracker {
public:
    TargetTracker();

    /**
     * @brief 使用视觉观测值更新滤波器（处理延迟数据）
     * @param obs 观测到的世界坐标 (x, y, z)
     * @param obs_ts 图像拍摄的原始纳秒时间戳
     */
    void update(const Eigen::Vector3d& obs, TimestampNS obs_ts);

    /**
     * @brief 预测目标在任意时刻的位置（用于补偿延迟和推算未来）
     * @param target_ts 想要预测到的目标时刻
     * @return 预测的状态
     */
    TrackerState predict(TimestampNS target_ts);

    /**
     * @brief 检查目标是否丢失过久
     */
    bool is_lost(TimestampNS now_ts, double threshold_sec = 2.0);

    // 重置滤波器
    void reset();

private:
    std::mutex mtx_;
    
    // 卡尔曼滤波器核心矩阵
    Eigen::VectorXd X_; // 状态向量 [x, y, z, vx, vy, vz]
    Eigen::MatrixXd P_; // 状态协方差
    Eigen::MatrixXd Q_; // 过程噪声
    Eigen::MatrixXd R_; // 测量噪声
    Eigen::MatrixXd H_; // 测量矩阵

    TimestampNS last_update_ts_; // 上次观测更新的时间
    bool is_initialized_;

    // 内部预测步骤（不改变状态，仅返回计算结果）
    Eigen::VectorXd predict_state(const Eigen::VectorXd& current_X, double dt);
};