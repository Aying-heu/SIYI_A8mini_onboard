#include "TargetTracker.hpp"

TargetTracker::TargetTracker() : is_initialized_(false) {
    reset();
}

void TargetTracker::reset() {
    std::lock_guard<std::mutex> lock(mtx_);
    X_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6) * 1.0;
    
    // 过程噪声 Q：代表物理模型的不确定性（假设目标加速度波动）
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
    
    // 测量噪声 R：代表 PnP 定位的抖动（根据你实测 Z 轴抖动大的情况调大 R）
    R_ = Eigen::MatrixXd::Identity(3, 3);
    R_(0,0) = 0.05; // X 较准
    R_(1,1) = 0.05; // Y 较准
    R_(2,2) = 0.5;  // Z 轴由于单目特性，给较大的噪声权重（信任模型而非观测）

    // 测量矩阵 H：我们将 6 维状态映射到 3 维位置观测
    H_ = Eigen::MatrixXd::Zero(3, 6);
    H_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    is_initialized_ = false;
    last_update_ts_ = 0;
}

void TargetTracker::update(const Eigen::Vector3d& obs, TimestampNS obs_ts) {
    std::lock_guard<std::mutex> lock(mtx_);

    // 1. 初始化处理
    if (!is_initialized_) {
        X_.head<3>() = obs;
        X_.tail<3>() = Eigen::Vector3d::Zero();
        last_update_ts_ = obs_ts;
        is_initialized_ = true;
        return;
    }

    // 2. 计算时间步长 (dt 为秒)
    double dt = static_cast<double>(obs_ts - last_update_ts_) * 1e-9;
    if (dt < 0) return; // 忽略乱序数据

    // --- 卡尔曼预测步骤 (Predict) ---
    // A 是状态转移矩阵
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
    A(0, 3) = A(1, 4) = A(2, 5) = dt;

    X_ = A * X_;
    P_ = A * P_ * A.transpose() + Q_;

    // --- 卡尔曼修正步骤 (Update) ---
    Eigen::Vector3d z = obs;
    // 关键修正：如果你确定目标在地面，可以在这里强制约束
    // z[2] = 0.0; 

    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    X_ = X_ + K * (z - H_ * X_);
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;

    last_update_ts_ = obs_ts;
}

TrackerState TargetTracker::predict(TimestampNS target_ts) {
    std::lock_guard<std::mutex> lock(mtx_);
    TrackerState result;
    
    if (!is_initialized_) return result;

    // 计算从“上次观测更新”到“目标时刻”的时间差（通常为 200ms 的延迟补偿）
    double dt = static_cast<double>(target_ts - last_update_ts_) * 1e-9;

    // 使用常速度模型外推
    result.pos = X_.head<3>() + X_.tail<3>() * dt;
    result.vel = X_.tail<3>();
    result.ts = target_ts;
    result.is_valid = true;

    return result;
}

bool TargetTracker::is_lost(TimestampNS now_ts, double threshold_sec) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!is_initialized_) return true;
    double dt = static_cast<double>(now_ts - last_update_ts_) * 1e-9;
    return dt > threshold_sec;
}