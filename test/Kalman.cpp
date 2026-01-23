// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <string>
// #include <algorithm>
// #include <Eigen/Dense>
// #include "TargetTracker.hpp"  // 引入你之前的跟踪器头文件
// #include "matplotlibcpp.h"     // 用于可视化（需要安装matplotlib-cpp）

// namespace plt = matplotlibcpp;

// // 定义时间戳类型（纳秒）
// using TimestampNS = uint64_t;

// // 存储CSV单行数据的结构体
// struct CSVData {
//     TimestampNS timestamp_ns;  // 主时间戳
//     double px;                 // X位置
//     double py;                 // Y位置
//     double pz;                 // Z位置
// };

// // 读取CSV文件并按时间戳排序
// std::vector<CSVData> readCSV(const std::string& filename) {
//     std::vector<CSVData> data_list;
//     std::ifstream file(filename);
//     if (!file.is_open()) {
//         std::cerr << "Error: 无法打开CSV文件 " << filename << std::endl;
//         return data_list;
//     }

//     std::string line;
//     // 跳过表头
//     std::getline(file, line);

//     // 逐行读取数据
//     while (std::getline(file, line)) {
//         std::istringstream ss(line);
//         std::string token;
//         CSVData data;

//         // 解析每一列（按CSV列顺序）
//         // 1. timestamp_ns
//         std::getline(ss, token, ',');
//         data.timestamp_ns = std::stoull(token);
        
//         // 2. dNano_t（跳过）
//         std::getline(ss, token, ',');
        
//         // 3. px
//         std::getline(ss, token, ',');
//         data.px = std::stod(token);
        
//         // 4. py
//         std::getline(ss, token, ',');
//         data.py = std::stod(token);
        
//         // 5. pz
//         std::getline(ss, token, ',');
//         data.pz = std::stod(token);
        
//         // 跳过剩余列（attitude相关）
//         while (std::getline(ss, token, ','));

//         data_list.push_back(data);
//     }
//     file.close();

//     // 按时间戳升序排序（确保数据时序正确）
//     std::sort(data_list.begin(), data_list.end(),
//         [](const CSVData& a, const CSVData& b) {
//             return a.timestamp_ns < b.timestamp_ns;
//         });

//     std::cout << "成功读取 " << data_list.size() << " 条CSV数据" << std::endl;
//     return data_list;
// }

// // 主函数：处理数据 + 卡尔曼滤波 + 可视化
// int main(int argc, char** argv) {
//     // 1. 读取CSV数据（替换为你的CSV文件路径）
//     std::string csv_path = "/home/bsa/A_vision_relate/data_20260114_153832/target_states.csv";
//     std::vector<CSVData> csv_data = readCSV(csv_path);
//     if (csv_data.empty()) {
//         return -1;
//     }

//     // 2. 初始化卡尔曼跟踪器
//     TargetTracker tracker;

//     // 3. 存储结果用于可视化
//     std::vector<double> time_list;          // 时间轴（秒）
//     std::vector<double> obs_x, obs_y, obs_z; // 原始观测值
//     std::vector<double> pred_x, pred_y, pred_z; // 卡尔曼预测值
//     std::vector<double> filter_x, filter_y, filter_z; // 卡尔曼滤波后的值

//     // 基准时间（转换为秒，方便可视化）
//     TimestampNS base_ts = csv_data[0].timestamp_ns;

//     // 4. 逐帧处理数据
//     for (const auto& data : csv_data) {
//         // 转换时间为相对于基准的秒数
//         double t = static_cast<double>(data.timestamp_ns - base_ts) * 1e-9;
//         time_list.push_back(t);

//         // 原始观测值
//         Eigen::Vector3d obs(data.px, data.py, data.pz);
//         obs_x.push_back(data.px);
//         obs_y.push_back(data.py);
//         obs_z.push_back(data.pz);

//         // 卡尔曼更新（传入观测值和时间戳）
//         tracker.update(obs, data.timestamp_ns);

//         // 预测当前时刻的状态（也可以预测未来时刻，比如+0.1秒）
//         TrackerState pred_state = tracker.predict(data.timestamp_ns);
        
//         // 存储预测值（如果有效）
//         if (pred_state.is_valid) {
//             pred_x.push_back(pred_state.pos.x());
//             pred_y.push_back(pred_state.pos.y());
//             pred_z.push_back(pred_state.pos.z());
//         } else {
//             pred_x.push_back(0.0);
//             pred_y.push_back(0.0);
//             pred_z.push_back(0.0);
//         }

//         // 额外：获取滤波后的状态（如果TargetTracker有getter函数，否则注释）
//         // 这里假设你给TargetTracker添加了获取X_的接口：Eigen::VectorXd getState()
//         // Eigen::VectorXd filter_state = tracker.getState();
//         // filter_x.push_back(filter_state(0));
//         // filter_y.push_back(filter_state(1));
//         // filter_z.push_back(filter_state(2));
//     }

//     // 5. 可视化结果（matplotlibcpp）
//     plt::figure_size(1200, 800);

//     // 绘制X轴位置对比
//     plt::subplot(3, 1, 1);
//     plt::plot(time_list, obs_x, "r.-", {"label", "原始观测X"});
//     plt::plot(time_list, pred_x, "b*-", {"label", "卡尔曼预测X"});
//     // plt::plot(time_list, filter_x, "g--", {"label", "卡尔曼滤波X"}); // 如果有滤波值
//     plt::title("X轴位置跟踪结果");
//     plt::xlabel("时间 (s)");
//     plt::ylabel("位置 (m)");
//     plt::legend();
//     plt::grid(true);

//     // 绘制Y轴位置对比
//     plt::subplot(3, 1, 2);
//     plt::plot(time_list, obs_y, "r.-", {"label", "原始观测Y"});
//     plt::plot(time_list, pred_y, "b*-", {"label", "卡尔曼预测Y"});
//     // plt::plot(time_list, filter_y, "g--", {"label", "卡尔曼滤波Y"});
//     plt::title("Y轴位置跟踪结果");
//     plt::xlabel("时间 (s)");
//     plt::ylabel("位置 (m)");
//     plt::legend();
//     plt::grid(true);

//     // 绘制Z轴位置对比
//     plt::subplot(3, 1, 3);
//     plt::plot(time_list, obs_z, "r.-", {"label", "原始观测Z"});
//     plt::plot(time_list, pred_z, "b*-", {"label", "卡尔曼预测Z"});
//     // plt::plot(time_list, filter_z, "g--", {"label", "卡尔曼滤波Z"});
//     plt::title("Z轴位置跟踪结果");
//     plt::xlabel("时间 (s)");
//     plt::ylabel("位置 (m)");
//     plt::legend();
//     plt::grid(true);

//     plt::tight_layout();
//     plt::show();

//     // 6. 可选：输出结果到CSV文件
//     std::ofstream out_file("tracking_result.csv");
//     out_file << "time_s,obs_x,obs_y,obs_z,pred_x,pred_y,pred_z\n";
//     for (int i = 0; i < time_list.size(); ++i) {
//         out_file << time_list[i] << ","
//                  << obs_x[i] << "," << obs_y[i] << "," << obs_z[i] << ","
//                  << pred_x[i] << "," << pred_y[i] << "," << pred_z[i] << "\n";
//     }
//     out_file.close();
//     std::cout << "跟踪结果已保存到 tracking_result.csv" << std::endl;

//     return 0;
// }







#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <Eigen/Dense>
#include "TargetTracker.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// 定义时间戳类型（纳秒）
using TimestampNS = uint64_t;

// 存储CSV单行数据的结构体
struct CSVData {
    TimestampNS timestamp_ns;  // 主时间戳
    double px;                 // X位置
    double py;                 // Y位置
    double pz;                 // Z位置
};

// 读取CSV文件并按时间戳排序
std::vector<CSVData> readCSV(const std::string& filename) {
    std::vector<CSVData> data_list;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: 无法打开CSV文件 " << filename << std::endl;
        return data_list;
    }

    std::string line;
    // 跳过表头
    std::getline(file, line);

    // 逐行读取数据
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        CSVData data;

        // 解析每一列（按CSV列顺序）
        // 1. timestamp_ns
        std::getline(ss, token, ',');
        data.timestamp_ns = std::stoull(token);
        
        // 2. dNano_t（跳过）
        std::getline(ss, token, ',');
        
        // 3. px
        std::getline(ss, token, ',');
        data.px = std::stod(token);
        
        // 4. py
        std::getline(ss, token, ',');
        data.py = std::stod(token);
        
        // 5. pz
        std::getline(ss, token, ',');
        data.pz = std::stod(token);
        
        // 跳过剩余列（attitude相关）
        while (std::getline(ss, token, ','));

        data_list.push_back(data);
    }
    file.close();

    // 按时间戳升序排序（确保数据时序正确）
    std::sort(data_list.begin(), data_list.end(),
        [](const CSVData& a, const CSVData& b) {
            return a.timestamp_ns < b.timestamp_ns;
        });

    std::cout << "成功读取 " << data_list.size() << " 条CSV数据" << std::endl;
    return data_list;
}

// 主函数：处理数据 + 卡尔曼滤波 + 可视化
int main(int argc, char** argv) {
    // 1. 读取CSV数据（替换为你的CSV文件路径）
    std::string csv_path = "/home/bsa/A_vision_relate/data_20260114_153832/target_states.csv";
    std::vector<CSVData> csv_data = readCSV(csv_path);
    if (csv_data.empty()) {
        return -1;
    }

    // 2. 初始化卡尔曼跟踪器
    TargetTracker tracker;

    // 3. 存储结果用于可视化
    std::vector<double> time_list;          // 时间轴（秒）
    std::vector<double> obs_x, obs_y, obs_z; // 原始观测值
    std::vector<double> pred_x, pred_y, pred_z; // 卡尔曼预测值

    // 基准时间（转换为秒，方便可视化）
    TimestampNS base_ts = csv_data[0].timestamp_ns;

    // 4. 逐帧处理数据
    for (const auto& data : csv_data) {
        // 转换时间为相对于基准的秒数
        double t = static_cast<double>(data.timestamp_ns - base_ts) * 1e-9;
        time_list.push_back(t);

        // 原始观测值
        Eigen::Vector3d obs(data.px, data.py, data.pz);
        obs_x.push_back(data.px);
        obs_y.push_back(data.py);
        obs_z.push_back(data.pz);

        // 卡尔曼更新（传入观测值和时间戳）
        tracker.update(obs, data.timestamp_ns);

        // 预测当前时刻的状态
        TrackerState pred_state = tracker.predict(data.timestamp_ns);
        
        // 存储预测值（如果有效）
        if (pred_state.is_valid) {
            pred_x.push_back(pred_state.pos.x());
            pred_y.push_back(pred_state.pos.y());
            pred_z.push_back(pred_state.pos.z());
        } else {
            pred_x.push_back(0.0);
            pred_y.push_back(0.0);
            pred_z.push_back(0.0);
        }
    }

    // // 5. 可视化结果（适配旧版本matplotlibcpp）
    // plt::figure_size(1200, 800);

    // // 绘制X轴位置对比
    // plt::subplot(3, 1, 1);
    // plt::plot(time_list, obs_x, "r.-");
    // plt::plot(time_list, pred_x, "b*-");
    // plt::title("X轴位置跟踪结果");
    // plt::xlabel("时间 (s)");
    // plt::ylabel("位置 (m)");
    // plt::legend({"原始观测X", "卡尔曼预测X"});
    // plt::grid(true);

    // // 绘制Y轴位置对比
    // plt::subplot(3, 1, 2);
    // plt::plot(time_list, obs_y, "r.-");
    // plt::plot(time_list, pred_y, "b*-");
    // plt::title("Y轴位置跟踪结果");
    // plt::xlabel("时间 (s)");
    // plt::ylabel("位置 (m)");
    // plt::legend({"原始观测Y", "卡尔曼预测Y"});
    // plt::grid(true);

    // // 绘制Z轴位置对比（修复报错行）
    // plt::subplot(3, 1, 3);
    // plt::plot(time_list, obs_z, "r.-");
    // plt::plot(time_list, pred_z, "b*-");
    // plt::title("Z轴位置跟踪结果");
    // plt::xlabel("时间 (s)");
    // plt::ylabel("位置 (m)");
    // plt::legend({"原始观测Z", "卡尔曼预测Z"});
    // plt::grid(true);

    // plt::tight_layout();
    // plt::show();

    // 6. 输出结果到CSV文件
    std::ofstream out_file("tracking_result.csv");
    out_file << "time_s,obs_x,obs_y,obs_z,pred_x,pred_y,pred_z\n";
    for (int i = 0; i < time_list.size(); ++i) {
        out_file << time_list[i] << ","
                 << obs_x[i] << "," << obs_y[i] << "," << obs_z[i] << ","
                 << pred_x[i] << "," << pred_y[i] << "," << pred_z[i] << "\n";
    }
    out_file.close();
    std::cout << "跟踪结果已保存到 tracking_result.csv" << std::endl;

    return 0;
}