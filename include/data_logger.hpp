#pragma once
#include "data_types.hpp"
#include "data_manager.hpp"
#include <fstream>
#include <thread>
#include <atomic>

namespace fs = std::filesystem;


class DataLogger {
public:
    DataLogger(std::shared_ptr<DataManager> data_manager);
    ~DataLogger();

    // 启动存储线程
    void start();
    // 停止存储线程
    void stop();

private:
    void run(); // 线程主循环

    std::shared_ptr<DataManager> data_manager_;
    std::thread log_thread_;
    std::atomic<bool> running_{false};

    // 文件流
    std::ofstream uav_file_;
    std::ofstream gimbal_file_;
    std::ofstream target_file_;

    void initialize_files();
    void close_files();
};