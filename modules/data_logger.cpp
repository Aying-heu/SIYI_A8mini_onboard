#include "data_logger.hpp"
#include "data_manager.hpp"

#include <yaml-cpp/yaml.h>
// sudo apt update && sudo apt install libyaml-cpp-dev
// 编译时需要链接库：g++ your_code.cpp -o your_code -lyaml-cpp

#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <filesystem>


namespace fs = std::filesystem;

DataLogger::DataLogger(std::shared_ptr<DataManager> data_manager)
    : data_manager_(data_manager) {
    initialize_files();
}

DataLogger::~DataLogger() {
    stop();
    close_files();
}

void DataLogger::initialize_files() {
    // 从 data_manager 获取本次运行的目录
    // 假设 DataManager 有一个返回 DataConfig 的接口
    auto config = data_manager_->get_config(); 
    std::string base_dir = config->base_dir_;

    // 1. 无人机状态文件
    uav_file_.open(config->uav_state_file_, std::ios::app);
    uav_file_ << "timestamp_ns,dNano_t,position_x,position_y,position_z,velocity_x,velocity_y,velocity_z,attitude_qx,attitude_qy,attitude_qz,attitude_qw,attitude_roll,attitude_pitch,attitude_yaw,thrust_1,thrust_2,thrust_3,thrust_4\n";

    // 2. 云台状态文件
    gimbal_file_.open(config->gimbal_state_file_, std::ios::app);
    gimbal_file_ << "timestamp_ns,dNano_t,zoom,roll,pitch,yaw,roll_velocity,pitch_velocity,yaw_velocity\n";

    // 3. 目标信息文件
    target_file_ori_.open(config->target_file_ori_, std::ios::app);
    target_file_ori_ << "timestamp_ns,dNano_t,px,py,pz,attitude0,attitude1,attitude2,attitude_q_x,attitude_q_y,attitude_q_z,attitude_q_w\n";
    target_file_filtered_.open(config->target_file_filtered_, std::ios::app);
    target_file_filtered_ << "timestamp_ns,dNano_t,px,py,pz,attitude0,attitude1,attitude2,attitude_q_x,attitude_q_y,attitude_q_z,attitude_q_w\n";
    target_file_15hz_.open(config->target_file_15hz_, std::ios::app);
    target_file_15hz_ << "timestamp_ns,dNano_t,px,py,pz,attitude0,attitude1,attitude2,attitude_q_x,attitude_q_y,attitude_q_z,attitude_q_w\n";
}

void DataLogger::start() {
    if (!running_.load()) {
        running_.store(true);
        log_thread_ = std::thread(&DataLogger::run, this);
    }
}

void DataLogger::stop() {
    if (running_.load()) {
        running_.store(false);
        if (log_thread_.joinable()) {
            log_thread_.join();
        }
    }
}

void DataLogger::run() {
    while (rclcpp::ok() && running_.load()) {
        // 准备本地容器，用来接收从 DataManager 中弹出的数据
        std::vector<UAV_Row> uav_list;
        std::vector<Gimbal_Row> gimbal_list;
        std::vector<Img_Row> img_list;
        std::vector<prometheus_msgs::msg::Target> target_list_ori;
        std::vector<prometheus_msgs::msg::Target> target_list_filtered;
        std::vector<TargetState> target_list_15hz;

        // 1. 从数据中心取出所有“已用”数据（内部执行 swap，极快）
        data_manager_->popUsedData(uav_list, gimbal_list, img_list, target_list_ori, target_list_filtered, target_list_15hz);

        // 2. 写入无人机数据
        TimestampNS uav_time=0;
        TimestampNS uav_dt=0;
        for (const auto& u : uav_list) {
            uav_dt=u.nano_timestamp-uav_time;
            uav_time=u.nano_timestamp;
            uav_file_ << uav_time << "," << uav_dt << ","
                      << u.position_x << "," << u.position_y << "," << u.position_z << ","
                      << u.velocity_x << "," << u.velocity_y << "," << u.velocity_z << ","
                      << u.attitude_qx << "," << u.attitude_qy << "," << u.attitude_qz << "," << u.attitude_qw << ","
                      << u.attitude_roll << "," << u.attitude_pitch << "," << u.attitude_yaw << ","
                      << u.thrust_1 << "," << u.thrust_2 << "," << u.thrust_3 << "," << u.thrust_3 <<"\n";
        }

        // 3. 写入云台数据
        TimestampNS gimbal_time = 0; 
        TimestampNS gimbal_dt=0;
        for (const auto& g : gimbal_list) {
            gimbal_dt=g.nano_timestamp-gimbal_time;
            gimbal_time=g.nano_timestamp;
            gimbal_file_ << gimbal_time << "," << gimbal_dt <<","
                         << g.zoom << "," << g.roll << "," << g.pitch << "," << g.yaw << ","
                         << g.roll_velocity << "," << g.pitch_velocity << "," << g.yaw_velocity << "\n";
        }

        // 4. 写入目标数据
        TimestampNS target_time = 0; 
        TimestampNS target_dt=0;
        for (auto& t : target_list_ori) {
            rclcpp::Time stamp_time(t.timestamp.stamp);
            target_dt=stamp_time.nanoseconds()-target_time;
            target_time = stamp_time.nanoseconds();
            target_file_ori_ << target_time << "," << target_dt << ","
                         << t.px << "," << t.py << "," << t.pz << ","
                         << t.attitude[0] << "," << t.attitude[1] << "," << t.attitude[2]<<","
                         << t.attitude_q.x << "," << t.attitude_q.y << "," << t.attitude_q.z << "," << t.attitude_q.w << "\n";
        }
        target_time = 0; 
        target_dt=0;
        for (auto& t : target_list_filtered) {
            rclcpp::Time stamp_time(t.timestamp.stamp);
            target_dt=stamp_time.nanoseconds()-target_time;
            target_time = stamp_time.nanoseconds();
            target_file_filtered_ << target_time << "," << target_dt << ","
                         << t.px << "," << t.py << "," << t.pz << ","
                         << t.attitude[0] << "," << t.attitude[1] << "," << t.attitude[2]<<","
                         << t.attitude_q.x << "," << t.attitude_q.y << "," << t.attitude_q.z << "," << t.attitude_q.w << "\n";
        }
        gimbal_time = 0; 
        target_dt=0;
        for (const auto& t : target_list_15hz) {
            target_dt=t.timestamp-target_time;
            target_time = t.timestamp;
            target_file_15hz_ << target_time << "," << target_dt <<","
                         << t.position[0] << "," << t.position[1] << "," << t.position[2] << ","
                         << t.angular_rad[0] << "," << t.angular_rad[1] << "," << t.angular_rad[2]<<","
                         << t.orientation.x() << "," << t.orientation.y() << "," << t.orientation.z() << "," << t.orientation.w() << "\n";
        }

        int idx = 0;
        int idx2=0;
        for (const auto& img_row : img_list) {
            idx++;
            if (img_row.img.empty()) continue;

            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 100};
            std::string filename = "/" + std::to_string(img_row.nano_timestamp) + ".jpg";

            if (img_row.is_detected) {
                idx2++;
                // 优先保存检测到的图
                std::string full_path1 = data_manager_->get_config()->image_dir_ + "/detected/ori/" + filename;
                std::string full_path2 = data_manager_->get_config()->image_dir_ + "/detected/draw/" + filename;
                cv::imwrite(full_path1, img_row.img, params);
                if(idx2%5==0){
                    cv::imwrite(full_path2, img_row.draw, params);
                }
            } else if (idx % 5 == 0) {
                // 如果没检测到，但满足抽样频率，保存到 raw
                std::string full_path = data_manager_->get_config()->image_dir_ + "/raw" + filename;
                cv::imwrite(full_path, img_row.img, params);
            }
        }

        // 6. 强制刷新缓冲区，确保异常断电时数据已落盘
        uav_file_.flush();
        gimbal_file_.flush();
        target_file_ori_.flush();
        target_file_filtered_.flush();
        target_file_15hz_.flush();

        uav_list.clear();
        gimbal_list.clear();
        img_list.clear();
        target_list_ori.clear();
        target_list_filtered.clear();
        target_list_15hz.clear();

        // 7. 降低频率，减轻 CPU 和磁盘压力 (比如 2Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void DataLogger::close_files() {
    if (uav_file_.is_open()) uav_file_.close();
    if (gimbal_file_.is_open()) gimbal_file_.close();
    if (target_file_ori_.is_open()) target_file_ori_.close();
    if (target_file_filtered_.is_open()) target_file_filtered_.close();
}