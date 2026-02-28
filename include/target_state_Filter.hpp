// #pragma once
// #include <Eigen/Dense>
// #include <Eigen/Geometry>
// #include <iostream>
// #include <cmath>
// #include "data_types.hpp"
// #include <rclcpp/rclcpp.hpp>
// #include <prometheus_msgs/msg/target.hpp>
// #include <prometheus_msgs/msg/target_array.hpp>

// class TargetStateFilter {
// public:
//     TargetStateFilter();
//     ~TargetStateFilter() = default;

//     // 初始化滤波器
//     void init(const TargetState& initial_state);

//     // [输入] 当视觉系统有数据时调用 (频率不定: 0.2Hz ~ 15Hz)
//     void update_measurement(uint64_t t_ns, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat);

//     // [输出] 定时器调用，获取当前时刻的预测状态 (固定 20Hz)
//     // 这是一个"只读"预测，不会改变滤波器内部的后验状态
//     TargetState get_predicted_state(uint64_t  query_time_ns);

//     bool is_initialized() const { return initialized_; }

// private:
//     bool initialized_ = false;
//     uint64_t base_timestamp_ns_ = 0;
//     uint64_t last_update_time_ns_ = 0.0;

//     // double convert_to_relative_seconds(uint64_t current_ns) {
//     //     if (is_first_frame_) {
//     //         base_timestamp_ns_ = current_ns;
//     //         is_first_frame_ = false;
//     //         return 0.0;
//     //     }
//     //     // 先做减法（防止溢出和精度丢失），再转 double，最后除以 1e9
//     //     return static_cast<double>(current_ns - base_timestamp_ns_) / 1.0e9;
//     // }

//     // --- 位置部分 (Linear KF) ---
//     // 状态: [px, py, pz, vx, vy, vz]
//     Eigen::VectorXd x_pos_; 
//     Eigen::MatrixXd P_pos_; 
//     Eigen::MatrixXd Q_pos_base_; // 过程噪声基底
//     Eigen::MatrixXd R_pos_;      // 观测噪声

//     // --- 姿态部分 (EKF) ---
//     // 状态: [qw, qx, qy, qz, wx, wy, wz] (7维)
//     Eigen::VectorXd x_att_; 
//     Eigen::MatrixXd P_att_;
//     Eigen::MatrixXd Q_att_base_; 
//     Eigen::MatrixXd R_att_;

//     // --- 内部辅助函数 ---
//     // 核心预测逻辑，根据 delta_t 推演状态
//     void predict_step(double dt, Eigen::VectorXd& x_p, Eigen::MatrixXd& P_p, 
//                       Eigen::VectorXd& x_a, Eigen::MatrixXd& P_a);
    
//     // 构建四元数的 Omega 矩阵
//     Eigen::Matrix4d get_omega_matrix(const Eigen::Vector3d& w);
    
//     // 构建 EKF 雅可比矩阵
//     Eigen::MatrixXd get_jacobian_F(const Eigen::VectorXd& x_att, double dt);
// };




#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>
#include "data_types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <prometheus_msgs/msg/target.hpp>
#include <prometheus_msgs/msg/target_array.hpp>




// struct TargetState {
//     uint64_t timestamp;
//     Eigen::Vector3d position;
//     Eigen::Vector3d velocity;
//     Eigen::Quaterniond orientation;
//     Eigen::Vector3d angular_velocity;
// };

class TargetStateFilter {
public:
    TargetStateFilter();

    void init(const TargetState& initial_state);
    
    // 核心接口
    void update_measurement(uint64_t t_ns, const Eigen::Vector3d& pos_meas, const Eigen::Quaterniond& quat_meas);
    TargetState get_predicted_state(uint64_t query_time_ns);

private:
    // 预测步骤
    void predict_step(double dt);
    
    // ESKF 特有的"误差注入" (把误差合并回名义状态)
    void inject_error_state(const Eigen::VectorXd& delta_x);

    // 辅助计算
    Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& v);

private:
    bool initialized_ = false;
    uint64_t last_update_time_ns_ = 0;

    // --- 名义状态 (Nominal State) ---
    // 这里的变量存储的是"当前最优估计值"，本身不包含高斯分布信息
    Eigen::Vector3d p_;         // 位置 (Global)
    Eigen::Vector3d v_;         // 速度 (Global)
    Eigen::Quaterniond q_;      // 姿态 (Global -> Body) 或 (Body -> Global) 取决于定义，这里采用 Body->Global
    Eigen::Vector3d w_;         // 角速度 (Body Frame)

    // --- 误差状态协方差 (Error State Covariance) ---
    // 维度 12x12: [dp(3), dv(3), dtheta(3), dw(3)]
    // dtheta 是旋转向量误差，定义为: q_true = q_nom * Exp(dtheta/2)
    Eigen::Matrix<double, 12, 12> P_;

    // 噪声矩阵
    Eigen::Matrix<double, 12, 12> Q_; // 过程噪声
    Eigen::Matrix<double, 6, 6> R_;   // 测量噪声 (3 pos + 3 rot_error)
};