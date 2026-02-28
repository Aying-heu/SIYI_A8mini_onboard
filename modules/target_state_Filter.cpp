// #include "target_state_Filter.hpp"

// // TargetStateFilter::TargetStateFilter() {
// //     // 1. 初始化位置部分 (6维)
// //     x_pos_ = Eigen::VectorXd::Zero(6);
// //     P_pos_ = Eigen::MatrixXd::Identity(6, 6) * 1.0; // 初始不确定性
    
// //     // 过程噪声 Q (假设恒定速度模型，加速度是噪声)
// //     Q_pos_base_ = Eigen::MatrixXd::Identity(6, 6) * 0.1; 
    
// //     // 观测噪声 R (视觉测量的位置噪声)
// //     R_pos_ = Eigen::MatrixXd::Identity(3, 3) * 0.1; // 假设测量误差在cm级

// //     // 2. 初始化姿态部分 (7维)
// //     x_att_ = Eigen::VectorXd::Zero(7);
// //     x_att_(0) = 1.0; // w=1 (单位四元数)
// //     P_att_ = Eigen::MatrixXd::Identity(7, 7) * 0.1;

// //     // --- 修正 Q 矩阵 ---
// //     // 我们假设模型是恒定角速度。
// //     // 四元数部分(前4维)的噪声主要来自数值误差，应设为极小值。
// //     // 角速度部分(后3维)的噪声代表角加速度的扰动。
// //     Q_att_base_ = Eigen::MatrixXd::Zero(7, 7);
    
// //     // 四元数部分给极小噪声防止矩阵奇异，但不应影响估计
// //     Q_att_base_.block(0, 0, 4, 4) = Eigen::MatrixXd::Identity(4, 4) * 1e-6; 
    
// //     // 角速度部分给适当噪声 (根据物体运动剧烈程度调整)
// //     // 如果物体转动平滑，0.1 - 1.0 是合理的；如果非常平稳，可以更小。
// //     Q_att_base_.block(4, 4, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * 0.1; 

// //     // 观测噪声 R (视觉测量的四元数噪声)
// //     R_att_ = Eigen::MatrixXd::Identity(4, 4) * 0.2;
// // }


// TargetStateFilter::TargetStateFilter() {
//     // ... 位置部分保持不变 ...

//     // 2. 初始化姿态部分
//     x_att_ = Eigen::VectorXd::Zero(7);
//     x_att_(0) = 1.0; 
//     P_att_ = Eigen::MatrixXd::Identity(7, 7) * 0.01; // 初始协方差稍微小一点

//     // --- 修正 Q 矩阵 ---
//     Q_att_base_ = Eigen::MatrixXd::Zero(7, 7);
//     // 四元数部分：稍微给大一点，允许一些非线性的模型偏差
//     Q_att_base_.block(0, 0, 4, 4) = Eigen::MatrixXd::Identity(4, 4) * 1e-5; 
    
//     // 角速度部分：这决定了你对"角加速度"的容忍度
//     // 0.1 ~ 1.0 是合理的，如果目标转动很灵活，设为 1.0
//     Q_att_base_.block(4, 4, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * 0.5; 

//     // --- 修正 R 矩阵 (关键！) ---
//     // 视觉测量的四元数通常比较准，噪声很小。
//     // 0.2 太大了！建议 0.001 ~ 0.01 之间。
//     // 如果测量抖动大，设 0.01 (std=0.1)；如果测量稳，设 0.001 (std=0.03)
//     R_att_ = Eigen::MatrixXd::Identity(4, 4) * 0.005; 
// }


// void TargetStateFilter::init(const TargetState& initial_state) {
//     last_update_time_ns_ = initial_state.timestamp;

//     // Pos
//     x_pos_ << initial_state.position, initial_state.velocity; // 假设初始速度已知或为0

//     // Att
//     Eigen::Quaterniond q = initial_state.orientation.normalized();
//     x_att_ << q.w(), q.x(), q.y(), q.z(), 
//               initial_state.angular_velocity;
    
//     initialized_ = true;
// }

// void TargetStateFilter::predict_step(double dt, 
//                                     Eigen::VectorXd& x_pos, Eigen::MatrixXd& P_pos, 
//                                     Eigen::VectorXd& x_att, Eigen::MatrixXd& P_att) {
//     if (dt <= 0) return;

//     // --- A. 位置预测 (Linear) ---
//     // F 矩阵
//     Eigen::MatrixXd F_pos = Eigen::MatrixXd::Identity(6, 6);
//     F_pos(0, 3) = dt; F_pos(1, 4) = dt; F_pos(2, 5) = dt;

//     // 状态外推
//     x_pos = F_pos * x_pos;
    
//     // 协方差外推: P = F*P*F^T + Q * dt (Q随时间累积)
//     P_pos = F_pos * P_pos * F_pos.transpose() + Q_pos_base_ * dt;

//     // --- B. 姿态预测 (EKF) ---
//     // 提取当前四元数和角速度
//     Eigen::Vector4d q_vec = x_att.head(4);
//     Eigen::Vector3d w_vec = x_att.tail(3);
    
//     // 1. 状态外推 (非线性)
//     // q_next = q + 0.5 * dt * Omega * q
//     Eigen::Matrix4d Omega = get_omega_matrix(w_vec);
//     Eigen::Vector4d q_next = (Eigen::Matrix4d::Identity() + 0.5 * dt * Omega) * q_vec;
//     q_next.normalize(); // 强制归一化

//     // 角速度假设不变
//     Eigen::Vector3d w_next = w_vec;

//     x_att.head(4) = q_next;
//     x_att.tail(3) = w_next;

//     // 2. 协方差外推 (雅可比)
//     Eigen::MatrixXd F_att = get_jacobian_F(x_att, dt); // 注意这里用新的还是旧的x均可，通常用旧的线性化
//     P_att = F_att * P_att * F_att.transpose() + Q_att_base_ * dt;
// }

// void TargetStateFilter::update_measurement(uint64_t t_ns, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat_meas) {
//     if (!initialized_) {
//         TargetState init_s;
//         init_s.timestamp = t_ns;
//         init_s.position = pos;
//         init_s.velocity = Eigen::Vector3d::Zero();
//         init_s.orientation = quat_meas;
//         init_s.angular_velocity = Eigen::Vector3d::Zero();
//         init(init_s);
//         return;
//     }

//     double dt = static_cast<double>(t_ns - last_update_time_ns_) / 1.0e9;
//     if (dt < 0) {
//         // 处理乱序数据：简单丢弃，或者只做更新不预测
//         std::cerr << "Warning: Out of order measurement received." << std::endl;
//         return;
//     }

//     // 1. 先预测到当前测量时刻
//     predict_step(dt, x_pos_, P_pos_, x_att_, P_att_);

//     // 2. 位置更新 (KF Update)
//     {
//         Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
//         H.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3); // 我们只观测位置

//         Eigen::Vector3d z = pos;
//         Eigen::Vector3d z_pred = H * x_pos_;
//         Eigen::Vector3d y = z - z_pred; // 残差

//         Eigen::MatrixXd S = H * P_pos_ * H.transpose() + R_pos_;
//         Eigen::MatrixXd K = P_pos_ * H.transpose() * S.inverse();

//         x_pos_ = x_pos_ + K * y;
//         P_pos_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_pos_;
//     }

//     // // 3. 姿态更新 (EKF Update)
//     // {
//     //     // 处理四元数双倍覆盖问题 (Double Cover)
//     //     // 确保测量的四元数与当前预测的四元数方向一致
//     //     Eigen::Vector4d q_pred_vec = x_att_.head(4);
//     //     Eigen::Vector4d q_meas_vec;
//     //     q_meas_vec << quat_meas.w(), quat_meas.x(), quat_meas.y(), quat_meas.z();

//     //     if (q_meas_vec.dot(q_pred_vec) < 0.0) {
//     //         q_meas_vec = -q_meas_vec;
//     //     }

//     //     Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 7);
//     //     H.block(0, 0, 4, 4) = Eigen::MatrixXd::Identity(4, 4); // 观测四元数

//     //     Eigen::Vector4d y = q_meas_vec - q_pred_vec;

//     //     Eigen::MatrixXd S = H * P_att_ * H.transpose() + R_att_;
//     //     Eigen::MatrixXd K = P_att_ * H.transpose() * S.inverse();

//     //     x_att_ = x_att_ + K * y;
        
//     //     // 更新后必须归一化四元数部分
//     //     Eigen::Vector4d q_final = x_att_.head(4);
//     //     q_final.normalize();
//     //     // x_att_.head(4) = q_final;

//     //     // P_att_ = (Eigen::MatrixXd::Identity(7, 7) - K * H) * P_att_;

//     //     if (q_final(0) < 0) {
//     //         q_final = -q_final;
//     //     }

//     //     x_att_.head(4) = q_final;

//     //     // --- 修正 2: 协方差更新使用 Joseph Form 保证对称性与正定性 ---
//     //     // 原代码: P = (I - KH)P 容易导致 P 变得不对称或负定
//     //     Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
//     //     Eigen::MatrixXd Temp = (I - K * H);
//     //     // 标准 KF 更新是 P = (I-KH)P，但在高维EKF中建议如下：
//     //     P_att_ = Temp * P_att_ * Temp.transpose() + K * R_att_ * K.transpose(); 

//     //     //P_att_ = (Eigen::MatrixXd::Identity(7, 7) - K * H) * P_att_;


//     // }



//     // 3. 姿态更新 (EKF Update)
//     {
//         // ... (Double Cover 处理保持不变) ...
//         Eigen::Vector4d q_pred_vec = x_att_.head(4);
//         Eigen::Vector4d q_meas_vec;
//         q_meas_vec << quat_meas.w(), quat_meas.x(), quat_meas.y(), quat_meas.z();

//         if (q_meas_vec.dot(q_pred_vec) < 0.0) {
//             q_meas_vec = -q_meas_vec;
//         }

//         Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 7);
//         H.block(0, 0, 4, 4) = Eigen::MatrixXd::Identity(4, 4); 

//         Eigen::Vector4d y = q_meas_vec - q_pred_vec;

//         Eigen::MatrixXd S = H * P_att_ * H.transpose() + R_att_;
//         Eigen::MatrixXd K = P_att_ * H.transpose() * S.inverse();

//         // 状态更新
//         x_att_ = x_att_ + K * y;
        
//         // --- 核心修正开始 ---
        
//         // 1. 归一化四元数
//         Eigen::Vector4d q_updated = x_att_.head(4);
//         double norm_val = q_updated.norm();
//         q_updated.normalize();
        
//         if (q_updated(0) < 0) q_updated = -q_updated; // 保持w正
//         x_att_.head(4) = q_updated;

//         // 2. 更新协方差 (Joseph Form)
//         Eigen::MatrixXd I7 = Eigen::MatrixXd::Identity(7, 7);
//         Eigen::MatrixXd Temp = (I7 - K * H);
//         P_att_ = Temp * P_att_ * Temp.transpose() + K * R_att_ * K.transpose(); 

//         // 3. 强行修正 P 矩阵 (Covariance Projection)
//         // 因为我们在 Step 1 做了归一化，我们需要把 P 矩阵中关于"模长"变化的不确定性消除。
//         // 构造投影矩阵 J = I - q*q^T (这是简化版，用于投影掉平行于q的分量)
        
//         // 完整的 7x7 投影矩阵
//         Eigen::MatrixXd J_norm = Eigen::MatrixXd::Identity(7, 7);
        
//         // 对于四元数部分 (前4维)，J = I(4) - (q*q^T) / (q^T*q)
//         // 此时 q 已经是归一化的，所以 J = I - q*q^T
//         J_norm.block(0,0,4,4) = Eigen::MatrixXd::Identity(4,4) - q_updated * q_updated.transpose();

//         // 将 P 投影回切平面： P = J * P * J^T
//         P_att_ = J_norm * P_att_ * J_norm.transpose();
        
//         // 还可以加一个小数值保护，防止对角线变为0或负数
//         for(int i=0; i<7; ++i) {
//              if(P_att_(i,i) < 1e-9) P_att_(i,i) = 1e-9;
//         }

//         // --- 核心修正结束 ---
//     }

//     last_update_time_ns_ = t_ns;
// }

// TargetState TargetStateFilter::get_predicted_state(uint64_t query_time_ns) {
//     if (!initialized_) return {};

//     double dt = static_cast<double>(query_time_ns - last_update_time_ns_) / 1.0e9;

//     // 复制当前内部状态，以免 query 影响滤波器本身的迭代
//     Eigen::VectorXd x_p = x_pos_;
//     Eigen::MatrixXd P_p = P_pos_;
//     Eigen::VectorXd x_a = x_att_;
//     Eigen::MatrixXd P_a = P_att_;

//     // 如果查询时间在未来，进行外推
//     // 如果查询时间就是当前或过去，这步会直接跳过或处理
//     if (dt > 1e-6) {
//         predict_step(dt, x_p, P_p, x_a, P_a);
//     }

//     // 组装结果
//     TargetState result;
//     result.timestamp = query_time_ns;
//     result.position = x_p.head(3);
//     result.velocity = x_p.tail(3);
    
//     result.orientation = Eigen::Quaterniond(x_a(0), x_a(1), x_a(2), x_a(3));

//     if (result.orientation.w() < 0) {
//         result.orientation.coeffs() = -result.orientation.coeffs();
//     }

//     result.angular_velocity = x_a.tail(3);


//     // 根据四元数计算欧拉角
//     Eigen::Matrix3d R_filtered = result.orientation.toRotationMatrix();
//     double f_yaw, f_pitch, f_roll;
//     double f_sy = sqrt(R_filtered(0,0)*R_filtered(0,0) + R_filtered(1,0)*R_filtered(1,0));
//     if (f_sy > 1e-6) {  // 非万向锁
//         f_yaw = atan2(R_filtered(1,0), R_filtered(0,0));
//         f_pitch = atan2(-R_filtered(2,0), f_sy);
//         f_roll = atan2(R_filtered(2,1), R_filtered(2,2));
//     } else {  // 万向锁
//         f_yaw = atan2(-R_filtered(0,1), R_filtered(1,1));
//         f_pitch = atan2(-R_filtered(2,0), f_sy);
//         f_roll = 0.0;
//     }

//     result.angular_rad = {f_roll, f_pitch, f_yaw};

//     return result;
// }

// Eigen::Matrix4d TargetStateFilter::get_omega_matrix(const Eigen::Vector3d& w) {
//     Eigen::Matrix4d Omega;
//     Omega << 0, -w.x(), -w.y(), -w.z(),
//              w.x(), 0, w.z(), -w.y(),
//              w.y(), -w.z(), 0, w.x(),
//              w.z(), w.y(), -w.x(), 0;
//     return Omega;
// }

// Eigen::MatrixXd TargetStateFilter::get_jacobian_F(const Eigen::VectorXd& x, double dt) {
//     // 7x7 矩阵
//     Eigen::MatrixXd F = Eigen::MatrixXd::Identity(7, 7);
    
//     Eigen::Vector4d q = x.head(4);
//     Eigen::Vector3d w = x.tail(3);

//     // 左上: dq/dq = I + 0.5*dt*Omega
//     Eigen::Matrix4d Omega = get_omega_matrix(w);
//     F.block(0, 0, 4, 4) = Eigen::Matrix4d::Identity() + 0.5 * dt * Omega;

//     // 右上: dq/dw = 0.5*dt*Xi(q)
//     Eigen::MatrixXd Xi(4, 3);
//     Xi << -q.x(), -q.y(), -q.z(),
//            q.w(), -q.z(),  q.y(),
//            q.z(),  q.w(), -q.x(),
//           -q.y(),  q.x(),  q.w();
    
//     F.block(0, 4, 4, 3) = 0.5 * dt * Xi;

//     // 左下: dw/dq = 0
//     // 右下: dw/dw = I (恒定角速度模型)
    
//     return F;
// }




#include "target_state_Filter.hpp"

TargetStateFilter::TargetStateFilter() {
    // 1. 初始化协方差 P (12x12)
    P_.setIdentity();
    P_.block<3, 3>(0, 0) *= 1.0;   // Pos
    P_.block<3, 3>(3, 3) *= 1.0;   // Vel
    P_.block<3, 3>(6, 6) *= 0.1;   // Angle Error (rad)
    P_.block<3, 3>(9, 9) *= 0.1;   // AngVel Error

    // 2. 初始化过程噪声 Q (12x12)
    Q_.setZero();
    // 假设恒定速度，速度的导数（加速度）是白噪声
    double acc_noise_std = 1;  // m/s^2 
    // Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * acc_noise_std * acc_noise_std; 
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.05; 
    
    // 假设恒定角速度，角速度的导数（角加速度）是白噪声
    double ang_acc_noise_std = 0.05; // rad/s^2
    Q_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * ang_acc_noise_std * ang_acc_noise_std;

    // 也可以给位置和角度本身加一点极小的噪声，增加鲁棒性
    Q_.block<3, 3>(0, 0) *= 1e-6;
    Q_.block<3, 3>(6, 6) *= 1e-6;

    // 3. 初始化观测噪声 R (6x6)
    // 这一点非常关键：这里不再是四元数噪声，而是"旋转向量误差"的噪声
    // 视觉里程计通常精度在 cm 级和 0.5度~2度 之间
    R_.setIdentity();
    // R_.block<3, 3>(0, 0) *= 0.05 * 0.05; // 5cm 位置误差
    R_.block<3, 3>(0, 0) *= 1;
    
    // 角度误差：假设 2度 (2 * pi / 180 = 0.035 rad)
    // double ang_meas_err = 0.035; 
    // double ang_meas_err = 0.05; 
    // R_.block<3, 3>(3, 3) *= ang_meas_err * ang_meas_err;
    R_.block<3, 3>(3, 3) *= 0.5;

    // 初始化状态
    p_.setZero();
    v_.setZero();
    q_.setIdentity();
    w_.setZero();
}

void TargetStateFilter::init(const TargetState& initial_state) {
    last_update_time_ns_ = initial_state.timestamp;
    p_ = initial_state.position;
    v_ = initial_state.velocity; // 如果初始没有速度，就设0
    q_ = initial_state.orientation.normalized();
    w_ = initial_state.angular_velocity;
    initialized_ = true;
}

Eigen::Matrix3d TargetStateFilter::skew_symmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
}

// void TargetStateFilter::predict_step(double dt) {
//     if (dt <= 0) return;

//     // --- 1. 名义状态外推 (Nominal State Integration) ---
//     // 位置: p = p + v*dt
//     p_ += v_ * dt;
//     // 速度: v = v (Constant Velocity Model)
    
//     // 姿态: q = q * Exp(w * dt * 0.5)
//     // 使用四元数微分解积分
//     Eigen::Vector3d w_dt = w_ * dt;
//     double w_norm = w_dt.norm();
//     Eigen::Quaterniond dq;
//     if (w_norm > 1e-8) {
//         // 罗德里格斯公式 / 四元数指数映射
//         dq = Eigen::Quaterniond(Eigen::AngleAxisd(w_norm, w_dt.normalized()));
//     } else {
//         // 近似
//         dq = Eigen::Quaterniond(1, w_dt.x()*0.5, w_dt.y()*0.5, w_dt.z()*0.5).normalized();
//     }
//     q_ = (q_ * dq).normalized(); // 姿态积分后必须归一化，这是ESKF中唯一需要归一化的地方

//     // 角速度: w = w (Constant Angular Velocity Model)

//     // --- 2. 误差协方差外推 (Error Covariance Propagation) ---
//     // Fx 是误差状态的转移矩阵 (12x12)
//     // 状态顺序: [dp, dv, dtheta, dw]
//     Eigen::Matrix<double, 12, 12> Fx = Eigen::Matrix<double, 12, 12>::Identity();

//     // Pos 由 Vel 影响
//     Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

//     // Angle Error 传播: dtheta_{k+1} = R(w*dt)^T * dtheta_k + J * dw * dt
//     // 这里的具体推导取决于误差定义是 Global 还是 Local。
//     // 在基于 Body Frame 的误差定义下 (Standard for ESKF):
//     // dtheta_new = dtheta_old - (w x dtheta_old) * dt + dw * dt
//     Eigen::Matrix3d rot_step = dq.toRotationMatrix().transpose(); // 近似为 I - [w*dt]x
//     // 或者更精确的：Fx.block<3, 3>(6, 6) = rot_step; 
//     // 对于恒定角速度模型，通常简化为:
//     Fx.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew_symmetric(w_) * dt;
//     Fx.block<3, 3>(6, 9) = Eigen::Matrix3d::Identity() * dt;

//     // Q 矩阵离散化 (简单乘以 dt，或者用 Q_d = F*Q*F^T*dt 等高阶方法)
//     // 这里采用一阶近似
//     Eigen::Matrix<double, 12, 12> Q_discrete = Q_ * dt;

//     // P = F * P * F^T + Q
//     P_ = Fx * P_ * Fx.transpose() + Q_discrete;
    
//     // 强制对称性
//     P_ = 0.5 * (P_ + P_.transpose());
// }


void TargetStateFilter::predict_step(double dt) {
    if (dt <= 0) return;

    // --- 新增：阻尼模型 - 线速度+角速度线性衰减 ---
    double damp_coeff = 0.98; // 阻尼系数（核心调参，0.95~0.99为宜）
    // 线速度衰减：v = v * 阻尼系数^dt（线性近似，dt极小时等价指数衰减）
    v_ = v_ * pow(damp_coeff, dt); 
    // 角速度衰减：和线速度一致，保证姿态旋转同步减速
    w_ = w_ * pow(damp_coeff, dt);
    // -------------------------------------------------

    // --- 原有名义状态积分逻辑（完全不变）---
    p_ += v_ * dt; // 用衰减后的速度预测位置
    // 姿态预测（用衰减后的角速度）
    Eigen::Vector3d w_dt = w_ * dt;
    double w_norm = w_dt.norm();
    Eigen::Quaterniond dq;
    if (w_norm > 1e-8) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(w_norm, w_dt.normalized()));
    } else {
        dq = Eigen::Quaterniond(1, w_dt.x()*0.5, w_dt.y()*0.5, w_dt.z()*0.5).normalized();
    }
    q_ = (q_ * dq).normalized();
    // --- 原有误差协方差外推逻辑（完全不变）---
    Eigen::Matrix<double, 12, 12> Fx = Eigen::Matrix<double, 12, 12>::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew_symmetric(w_) * dt;
    Fx.block<3, 3>(6, 9) = Eigen::Matrix3d::Identity() * dt;
    Eigen::Matrix<double, 12, 12> Q_discrete = Q_ * dt;
    P_ = Fx * P_ * Fx.transpose() + Q_discrete;
    P_ = 0.5 * (P_ + P_.transpose());
}


void TargetStateFilter::update_measurement(uint64_t t_ns, const Eigen::Vector3d& pos_meas, const Eigen::Quaterniond& quat_meas) {
    if (!initialized_) {
        // 这里需要构建一个临时的 State 用于初始化
        TargetState init_s;
        init_s.timestamp = t_ns;
        init_s.position = pos_meas;
        init_s.velocity = Eigen::Vector3d::Zero();
        init_s.orientation = quat_meas;
        init_s.angular_velocity = Eigen::Vector3d::Zero();
        init(init_s);
        return;
    }

    double dt = (double)(t_ns - last_update_time_ns_) / 1e9;
    if (dt > 0) {
        predict_step(dt);
    }
    last_update_time_ns_ = t_ns;

    // --- 3. 测量更新 (Measurement Update) ---
    
    // 构建观测矩阵 H (6x12)
    // 我们观测的是 [pos, orientation]
    // 误差状态: [dp, dv, dtheta, dw]
    // 观测残差 y = z - h(x)
    // H = dy / d(error_state)
    
    Eigen::Matrix<double, 6, 12> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // Pos 误差直接对应 Pos 观测
    H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity(); // Angle 误差直接对应 Angle 观测 (在切空间)

    // 计算残差 (Residual)
    Eigen::VectorXd y(6);
    
    // 1. 位置残差: z_pos - x_pos
    y.head(3) = pos_meas - p_;

    // 2. 姿态残差 (重点!)
    // 我们需要计算 "measured_q" 和 "predicted_q" 之间的差异，并转换到轴角误差
    // delta_q = q_pred^-1 * q_meas
    Eigen::Quaterniond q_pred_inv = q_.conjugate();
    Eigen::Quaterniond q_error = q_pred_inv * quat_meas; // Local error
    q_error.normalize();

    // 转换四元数误差为旋转向量误差 (Axis-Angle)
    // dtheta = 2 * vec(q_error) / w(q_error) (小角度近似)
    // 或者标准 log 映射
    Eigen::AngleAxisd angle_axis(q_error);
    Eigen::Vector3d rot_error = angle_axis.angle() * angle_axis.axis();
    
    // 处理 +/- 180 度问题，确保寻找最小旋转路径
    if (rot_error.norm() > M_PI) {
       // 实际上 AngleAxis 已经处理了大部分，但为了保险：
       // 这里通常不需要额外处理，除非测量完全反向
    }
    y.tail(3) = rot_error;

    // 标准卡尔曼更新步骤
    Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R_;
    Eigen::Matrix<double, 12, 6> K = P_ * H.transpose() * S.inverse();

    // 计算误差状态 delta_x
    Eigen::VectorXd delta_x = K * y;

    // 更新协方差 (Joseph form to guarantee symmetry and PSD)
    Eigen::Matrix<double, 12, 12> I = Eigen::Matrix<double, 12, 12>::Identity();
    // P_ = (I - K * H) * P_; // 简单写法
    Eigen::Matrix<double, 12, 12> Temp = (I - K * H);
    P_ = Temp * P_ * Temp.transpose() + K * R_ * K.transpose();

    // --- 4. 误差注入 (Inject Error to Nominal) ---
    inject_error_state(delta_x);
}

void TargetStateFilter::inject_error_state(const Eigen::VectorXd& delta_x) {
    // 1. 位置更新
    p_ += delta_x.head(3);

    // 2. 速度更新
    v_ += delta_x.segment<3>(3);

    // 3. 姿态更新 (核心)
    // q_true = q_nom * Exp(dtheta / 2)
    Eigen::Vector3d dtheta = delta_x.segment<3>(6);
    Eigen::Quaterniond dq;
    double theta_norm = dtheta.norm();
    if (theta_norm > 1e-8) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(theta_norm, dtheta.normalized()));
    } else {
        dq = Eigen::Quaterniond(1, dtheta.x()*0.5, dtheta.y()*0.5, dtheta.z()*0.5);
        dq.normalize();
    }
    
    // 注意乘法顺序，取决于误差定义是左乘还是右乘
    // 这里的推导基于 Local Error: q_true = q_nom * dq
    q_ = (q_ * dq).normalized();

    // 4. 角速度更新
    w_ += delta_x.tail(3);

    // 5. 重置误差状态
    // 在 ESKF 中，每次 update 后误差状态都被合并进名义状态，
    // 因此误差状态的期望值归零。
    // (P 矩阵不需要重置，因为它代表的是"合并后的名义状态的不确定性")
}

// TargetState TargetStateFilter::get_predicted_state(uint64_t query_time_ns) {
//     if (!initialized_) return {};

//     double dt = (double)(query_time_ns - last_update_time_ns_) / 1e9;

//     // 这是一个只读预测，不改变内部状态
//     TargetState pred;
//     pred.timestamp = query_time_ns;
    
//     // 线性外推
//     pred.position = p_ + v_ * dt;
//     pred.velocity = v_; // 恒定速度
    
//     // 姿态外推
//     Eigen::Vector3d w_dt = w_ * dt;
//     Eigen::Quaterniond dq(Eigen::AngleAxisd(w_dt.norm(), w_dt.normalized()));
//     if (w_dt.norm() < 1e-8) dq.setIdentity();
    
//     pred.orientation = (q_ * dq).normalized();
//     pred.angular_velocity = w_;

//     // 填充欧拉角等其他辅助信息（如果TargetState结构体有定义的话）
//     // ...

//     return pred;
// }

TargetState TargetStateFilter::get_predicted_state(uint64_t query_time_ns) {
    if (!initialized_) return {};

    double dt = (double)(query_time_ns - last_update_time_ns_) / 1e9;
    
    // 这是一个只读预测，不改变内部状态
    TargetState pred;
    pred.timestamp = query_time_ns;


    // ================== 新增：阻尼模型（仅作用于局部预测变量，不修改原始v_/w_）==================
    double damp_coeff = 0.98; // 和predict_step中一致，建议提为类成员变量统一管理
    double decay_factor = pow(damp_coeff, dt); // 未来dt时长的总衰减因子（仅计算1次）
    Eigen::Vector3d v_damp = v_ * decay_factor; // 衰减后的速度（局部变量，不碰原始v_）
    Eigen::Vector3d w_damp = w_ * decay_factor; // 衰减后的角速度（局部变量，不碰原始w_）
    // ==========================================================================================


    // // 线性外推
    // pred.position = p_ + v_ * dt;
    // pred.velocity = v_; // 恒定速度

    // 线性外推【修改】：使用阻尼衰减后的局部速度v_damp
    pred.position = p_ + v_damp * dt;
    pred.velocity = v_damp; // 预测的速度为衰减后的值，更贴合实际
    
    // // 姿态外推
    // Eigen::Vector3d w_dt = w_ * dt;
    // Eigen::Quaterniond dq(Eigen::AngleAxisd(w_dt.norm(), w_dt.normalized()));
    // if (w_dt.norm() < 1e-8) dq.setIdentity();
    // pred.orientation = (q_ * dq).normalized();
    // pred.angular_velocity = w_;

    // 姿态外推【修改】：使用阻尼衰减后的局部角速度w_damp
    Eigen::Vector3d w_dt = w_damp * dt;
    Eigen::Quaterniond dq;
    if (w_dt.norm() > 1e-8) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(w_dt.norm(), w_dt.normalized()));
    } else {
        dq.setIdentity(); // 角速度极小时，姿态无旋转
    }
    pred.orientation = (q_ * dq).normalized();
    pred.angular_velocity = w_damp; // 预测的角速度为衰减后的值

    // ================== 新增：四元数转欧拉角 (RPY) ==================
    // 顺序通常为: Z-Y-X (Yaw-Pitch-Roll) 或者是你是用的其他顺序
    // 这里使用最通用的数学公式计算，范围 [-PI, PI]
    
    Eigen::Matrix3d R = pred.orientation.toRotationMatrix();
    double sy = std::sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    
    bool singular = sy < 1e-5; // 检查万向锁 (Gimbal Lock)

    double r, p, y;
    if (!singular) {
        r = std::atan2(R(2,1), R(2,2)); // Roll
        p = std::atan2(-R(2,0), sy);    // Pitch
        y = std::atan2(R(1,0), R(0,0)); // Yaw
    } else {
        r = std::atan2(-R(1,2), R(1,1));
        p = std::atan2(-R(2,0), sy);
        y = 0; // 万向锁时，Yaw 和 Roll 无法区分，通常设 Yaw=0
    }

    while (y > M_PI/2.0) y -=  M_PI;
    while (y < -M_PI/2.0) y +=  M_PI;

    while (p > M_PI/2.0) p -=  M_PI;
    while (p < -M_PI/2.0) p +=  M_PI;

    while (r > M_PI/2.0) r -=  M_PI;
    while (r < -M_PI/2.0) r +=  M_PI;

    // 假设你的 TargetState 结构体里有用 vector 或单独变量存储欧拉角
    // 请根据你的实际定义修改下面这行：
    pred.angular_rad = {r, p, y};  // 如果是 std::vector
    // 或者
    // pred.roll = r; pred.pitch = p; pred.yaw = y; 

    // =============================================================

    return pred;
}