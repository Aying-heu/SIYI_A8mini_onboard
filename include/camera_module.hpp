#pragma once
#include <opencv2/opencv.hpp>
#include "data_manager.hpp"
#include <thread>
#include <atomic>

class CameraModule {
public:
    CameraModule(std::shared_ptr<DataManager> data_manager, int device_id);
    ~CameraModule();

    bool start(); // 启动抓图线程
    void stop();

private:
    void run(); // 抓图线程主循环
    std::shared_ptr<DataManager> data_manager_;
    cv::VideoCapture cap_;
    std::thread camera_thread_;
    std::atomic<bool> running_{false};
    int device_id_;
};