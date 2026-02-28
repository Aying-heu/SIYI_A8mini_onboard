#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include "target_state_Filter.hpp"

// 辅助函数：将 ROS 消息格式转为滤波器需要的 TargetState 格式
TargetState msgToTargetState(const prometheus_msgs::msg::Target& msg, uint64_t t_sec) {
    TargetState state;
    state.timestamp = t_sec;
    state.position = Eigen::Vector3d(msg.px, msg.py, msg.pz);
    state.velocity = Eigen::Vector3d::Zero(); // 初始测量通常无速度
    state.orientation = Eigen::Quaterniond(
        msg.attitude_q.w, msg.attitude_q.x, msg.attitude_q.y, msg.attitude_q.z
    );
    state.angular_velocity = Eigen::Vector3d::Zero();
    return state;
}

// 你提供的读取函数
std::vector<prometheus_msgs::msg::Target> loadOfflineTargetStates(const std::string& target_state_file) {
    std::vector<prometheus_msgs::msg::Target> states;
    std::ifstream file(target_state_file);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << target_state_file << std::endl;
        return states;
    }

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

        if (columns.size() < 12) continue; // 确保包含 attitude_q_w

        try {
            prometheus_msgs::msg::Target row;
            // 处理时间戳 (取第一列)
            uint64_t ts_ns = std::stoull(columns[0]);
            row.timestamp.stamp = rclcpp::Time(ts_ns);
            
            row.px = std::stof(columns[2]);
            row.py = std::stof(columns[3]);
            row.pz = std::stof(columns[4]);
            
            row.attitude[0] = std::stof(columns[5]);
            row.attitude[1] = std::stof(columns[6]);
            row.attitude[2] = std::stof(columns[7]);
            
            // CSV 顺序: x, y, z, w
            row.attitude_q.x = std::stof(columns[8]);
            row.attitude_q.y = std::stof(columns[9]);
            row.attitude_q.z = std::stof(columns[10]);
            row.attitude_q.w = std::stof(columns[11]);
            
            states.push_back(row);
        } catch (...) { continue; }
    }
    return states;
}

int main(int argc, char** argv) {
    std::string input_file = "/home/bsa/A_vision_relate/STAR/0124_12_炸机/target_states.csv";
    std::string output_file = "/home/bsa/A_vision_relate/STAR/0124_12_炸机/target_states_filtered.csv";

    // 1. 加载数据
    std::vector<prometheus_msgs::msg::Target> measurements = loadOfflineTargetStates(input_file);
    if (measurements.empty()) {
        std::cout << "No data loaded." << std::endl;
        return -1;
    }
    std::cout << "Loaded " << measurements.size() << " data points." << std::endl;

    // 2. 初始化滤波器
    TargetStateFilter filter;
    
    // 准备输出文件
    std::ofstream out(output_file);
    out << "timestamp_ns,dNano_t,px,py,pz,attitude0,attitude1,attitude2,attitude_q_x,attitude_q_y,attitude_q_z,attitude_q_w\n";
    out << std::fixed << std::setprecision(6);

    uint64_t start_ts_ns = rclcpp::Time(measurements[0].timestamp.stamp).nanoseconds();

    for (const auto& msg : measurements) {
        uint64_t current_ts_ns = rclcpp::Time(msg.timestamp.stamp).nanoseconds();
        // 将纳秒转为相对秒（防止 double 精度丢失起始大数）
        uint64_t t_sec = (current_ts_ns - start_ts_ns) / 1e9;

        Eigen::Vector3d pos_meas(msg.px, msg.py, msg.pz);
        Eigen::Quaterniond quat_meas(msg.attitude_q.w, msg.attitude_q.x, msg.attitude_q.y, msg.attitude_q.z);

        // 3. 执行滤波更新
        filter.update_measurement(current_ts_ns, pos_meas, quat_meas);

        // 4. 获取滤波后的当前状态 (Query)
        TargetState filtered_res = filter.get_predicted_state(current_ts_ns);

        // 5. 写入文件
        out << current_ts_ns << "," << t_sec*1e9 << ","
            << filtered_res.position.x() << "," << filtered_res.position.y() << "," << filtered_res.position.z() << ","
            << filtered_res.orientation.w() << "," << filtered_res.orientation.x() << "," << filtered_res.orientation.y() << "," << filtered_res.orientation.z() << ","
            << filtered_res.angular_velocity.x() << "," << filtered_res.angular_velocity.y() << "," << filtered_res.angular_velocity.z() << "\n";
    }

    out.close();
    std::cout << "Filtering completed. Results saved to " << output_file << std::endl;

    return 0;
}