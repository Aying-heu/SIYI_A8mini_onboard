#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>

using TimestampNS = uint64_t; 

// 定义CSV每行数据的结构体
struct UAV_Row {
    TimestampNS nano_timestamp;    // 第一列：ros时间戳（整数）
    TimestampNS dNano_t;           // 第二列：dRos_t（整数）
    double position_x;          // 第三列：x坐标
    double position_y;          // 第四列：y坐标
    double position_z;          // 第五列：z坐标
    double velocity_x;          // 第六列：x方向速度
    double velocity_y;          // 第七列：y方向速度
    double velocity_z;          // 第八列：z方向速度
    double attitude_qx;         // 第九列：姿态四元数x
    double attitude_qy;         // 第十列：姿态四元数y
    double attitude_qz;         // 第十一列：姿态四元数z
    double attitude_qw;         // 第十二列：姿态四元数w
    double attitude_roll;       // 第十三列：横滚角
    double attitude_pitch;      // 第十四列：俯仰角
    double attitude_yaw;        // 第十五列：偏航角
    double thrust_1;
    double thrust_2;
    double thrust_3;
    double thrust_4;
};

struct Img_Row{
    TimestampNS nano_timestamp;
    TimestampNS dNano_t;
    cv::Mat img;
    std::string img_filename;
    std::string timestamp_str;  
    bool is_detected=false;
};

struct Gimbal_Row{
    TimestampNS nano_timestamp;
    TimestampNS dNano_t;
    double zoom;
    double roll;
    double pitch;
    double yaw;
    double roll_velocity;
    double pitch_velocity;
    double yaw_velocity;
};

struct DataConfig {
    std::string root_path_; // 数据根路径

    std::string cam_param_file_;    // 相机参数文件
    std::string zoom_correspond_file_;         // 变焦对应文件
    std::string WorldPoints_correspind_file_;         // WorldPoints_BasedOn_camera_img
    std::vector<std::string> model_path_;
    std::vector<std::vector<std::string>> classes_;

    std::string base_dir_; // 本次运行带时间戳的目录
    std::string uav_state_file_;    // UAV状态文件
    std::string gimbal_state_file_; // 云台状态文件
    std::string target_file_;
    std::string image_dir_;  // 图像目录前缀
};