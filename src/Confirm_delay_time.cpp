#include "data_types.hpp"
#include "data_manager.hpp"
#include "data_logger.hpp"
#include "camera_module.hpp"
#include "gimbal_module.hpp"
#include "vision_onnx_aruco.hpp"
#include "uav_module.hpp"
#include "target_state_Predictor.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"

// --------------- 新增：离线数据读取工具函数（主函数内/外部均可）---------------
// 1. 读取本地图片列表（按序）
std::vector<std::string> loadOfflineImageList(const std::string& img_dir) {
    std::vector<std::string> img_files;
    
    // 定义两个子目录路径
    std::string img_dir1 = img_dir + "/detected/ori";
    std::string img_dir2 = img_dir + "/raw/";

    // 辅助函数：读取单个目录下的图片并添加到列表
    auto load_single_dir = [&](const std::string& dir_path) {
        if (!std::filesystem::exists(dir_path)) {
            std::cerr << "警告：离线图片子目录不存在: " << dir_path << "，跳过该目录" << std::endl;
            return;
        }
        // 遍历当前目录下的图片文件
        for (const auto& entry : std::filesystem::directory_iterator(dir_path)) {
            // 跳过目录（只处理文件）
            if (!entry.is_regular_file()) continue;
            
            std::string ext = entry.path().extension().string();
            // 筛选图片格式
            if (ext == ".jpg" || ext == ".png" || ext == ".bmp") {
                img_files.push_back(entry.path().string());
            }
        }
    };

    // 1. 加载 detected 目录下的图片
    // 2. 加载 raw 目录下的图片
    load_single_dir(img_dir2);
    load_single_dir(img_dir1);

    // 3. 对合并后的图片列表按文件名（时间戳）排序
    if (!img_files.empty()) {
        std::sort(img_files.begin(), img_files.end());
    } else {
        std::cerr << "错误：detected 和 raw 目录下均未找到图片文件！" << std::endl;
    }

    return img_files;
}

// 1. 读取无人机状态（严格对应 19 列）
std::vector<UAV_Row> loadOfflineUavStates(const std::string& uav_state_file) {
    std::vector<UAV_Row> states;
    std::ifstream file(uav_state_file);
    std::string line;
    
    std::getline(file, line); // 跳过表头

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> columns;
        
        while (std::getline(ss, token, ',')) {
            columns.push_back(token);
        }

        if (columns.size() < 19) continue; // 确保列数完整

        try {
            UAV_Row row;
            // 时间戳：必须 stoull
            row.nano_timestamp = std::stoull(columns[0]); 
            
            // 位置
            row.position_x = std::stof(columns[2]);
            row.position_y = std::stof(columns[3]);
            row.position_z = std::stof(columns[4]);

            // 速度
            row.velocity_x = std::stof(columns[5]);
            row.velocity_y = std::stof(columns[6]);
            row.velocity_z = std::stof(columns[7]);

            // 四元数 (qx, qy, qz, qw)
            row.attitude_qx = std::stof(columns[8]);
            row.attitude_qy = std::stof(columns[9]); // 之前漏掉了这一列
            row.attitude_qz = std::stof(columns[10]);
            row.attitude_qw = std::stof(columns[11]);

            // 欧拉角 (roll, pitch, yaw)
            row.attitude_roll  = std::stof(columns[12]);
            row.attitude_pitch = std::stof(columns[13]);
            row.attitude_yaw   = std::stof(columns[14]);

            // 推力 (1, 2, 3, 4)
            row.thrust_1 = std::stof(columns[15]);
            row.thrust_2 = std::stof(columns[16]);
            row.thrust_3 = std::stof(columns[17]);
            row.thrust_4 = std::stof(columns[18]);

            states.push_back(row);
        } catch (...) { continue; }
    }
    return states;
}

// 2. 读取云台状态（严格对应 9 列）
std::vector<Gimbal_Row> loadOfflineGimbalStates(const std::string& gimbal_state_file) {
    std::vector<Gimbal_Row> states;
    std::ifstream file(gimbal_state_file);
    std::string line;

    std::getline(file, line); // 跳过表头

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> columns;

        while (std::getline(ss, token, ',')) {
            columns.push_back(token);
        }

        if (columns.size() < 9) continue;

        try {
            Gimbal_Row row;
            row.nano_timestamp = std::stoull(columns[0]);
            row.zoom           = std::stof(columns[2]);
            row.roll           = std::stof(columns[3]);
            row.pitch          = std::stof(columns[4]);
            row.yaw            = std::stof(columns[5]);
            row.roll_velocity  = std::stof(columns[6]);
            row.pitch_velocity = std::stof(columns[7]);
            row.yaw_velocity   = std::stof(columns[8]);
            states.push_back(row);
        } catch (...) { continue; }
    }
    return states;
}

uint64_t getTimestampFromFilename(const std::string& filepath) {
    std::string filename = std::filesystem::path(filepath).stem().string();
    try {
        // 仅保留数字部分，防止文件名有后缀或其他杂质
        std::string cleaned;
        for(char c : filename) if(std::isdigit(c)) cleaned += c;
        
        if(cleaned.empty()) return 0;
        return std::stoull(cleaned);
    } catch (...) {
        return 0;
    }
}

// --------------- 主函数核心逻辑 ---------------
int main(int argc, char** argv) {

    std::string offline_data_root="/home/bsa/A_vision_relate/STAR/0124_07_原地无人机旋转_云台锁定/";

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("uav_vision_system");

    // 2. 初始化DataManager（原有逻辑，无修改）
    auto data_manager = std::make_shared<DataManager>(node,"/home/bsa/A_vision_relate/");
    std::cout<<"data_manager started successful "<<std::endl;
        
    // 3. 初始化DataLogger（原有逻辑，无修改）
    auto data_logger = std::make_shared<DataLogger>(data_manager);
    data_logger->start();
    std::cout<<"data_logger started successful"<<std::endl;

    auto vision_module = std::make_shared<VisionModule>(data_manager);
    if(!vision_module ->start_process()){
        std::cout << "检测线程开启失败"<<std::endl;
        return 0;
    }
    std::cout<<"vision_module started successful"<<std::endl;

    auto predict_module = std::make_shared<TargetStatePredictor>(data_manager);
    // if(!predict_module ->start()){
    //     std::cout << "predict 线程开启失败"<<std::endl;
    //     return 0;
    // }
    // std::cout<<"predict_module started successful"<<std::endl;


    // ---------------- 离线模式：主函数直接推送本地数据 ----------------
    std::cout << "进入离线复盘模式，开始读取本地数据..." << std::endl;

    // 6.1 加载离线数据
    std::string img_dir = offline_data_root + "/images/";
    std::string uav_state_file = offline_data_root + "/uav_states.csv";
    std::string gimbal_state_file = offline_data_root + "/gimbal_states.csv";

    auto img_files = loadOfflineImageList(img_dir);
    auto uav_states = loadOfflineUavStates(uav_state_file);
    auto gimbal_states = loadOfflineGimbalStates(gimbal_state_file);

    if (uav_states.empty() || img_files.empty()) {
        std::cerr << "错误：离线数据加载失败，请检查路径！" << std::endl;
        return -1;
    }
    std::cout<<"加载图片个数："<<img_files.size()<<std::endl;

    // ---------------- 核心逻辑修改：时间轴驱动 ----------------

    // 获取起始和结束时间戳
    uint64_t start_ts = uav_states.front().nano_timestamp;
    uint64_t end_ts = uav_states.back().nano_timestamp;
    uint64_t step_ns = 5000000; // 20ms 步进

    size_t uav_ptr = 0;
    size_t gimbal_ptr = 0;
    size_t img_ptr = 0;

    std::cout << "开始时间轴复盘: " << start_ts << " -> " << end_ts << std::endl;
    std::cout << "步长: 5ms" << std::endl;

    for (uint64_t cur_ts = start_ts; cur_ts <= end_ts; cur_ts += step_ns) {
        
        // 1. 推送当前时刻之前的所有 UAV 状态
        while (uav_ptr < uav_states.size() && uav_states[uav_ptr].nano_timestamp <= cur_ts) {
            data_manager->pushUavState(uav_states[uav_ptr]);
            uav_ptr++;
        }

        // 2. 推送当前时刻之前的所有 Gimbal 状态
        while (gimbal_ptr < gimbal_states.size() && gimbal_states[gimbal_ptr].nano_timestamp <= cur_ts) {
            data_manager->pushGimbalState(gimbal_states[gimbal_ptr]);
            gimbal_ptr++;
        }

        // 3. 推送落在当前 20ms 时间窗内的所有图像
        // 逻辑：如果图片的拍摄时间在 (cur_ts - step_ns) 到 cur_ts 之间，则推送
        while (img_ptr < img_files.size()) {
            uint64_t img_ts = getTimestampFromFilename(img_files[img_ptr]);
            
            if (img_ts > cur_ts) {
                // 图片时间还没到，等下一轮时间轴步进
                break;
            }

            // 处理图片
            cv::Mat raw_img = cv::imread(img_files[img_ptr]);
            if (!raw_img.empty()) {
                cv::Mat resized_img;
                // 统一缩放到你的处理分辨率
                cv::resize(raw_img, resized_img, cv::Size(1280, 720)); 
                
                // 推送图像，使用原始拍摄时间戳
                data_manager->pushRawImage(resized_img, img_ts);
                
                std::cout << "[Replay] Pushed Image TS: " << img_ts << " at Sim Time: " << cur_ts << std::endl;
            }
            img_ptr++;
        }

        predict_module->tick(cur_ts); 

        // 4. 模拟回放速度 (可选)
        // 如果想“嗖”的一下跑完，注释掉下面这行
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // 5. 让 ROS 处理回调（虽然离线模式下主要是推送）
        if (!rclcpp::ok()) break;
        rclcpp::spin_some(node);
    }

    std::cout << "复盘完成。" << std::endl;
    rclcpp::shutdown();
    return 0;
}