#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main() {
    // 1. 构建针对 Xavier 优化的 GStreamer 管道
    // 假设你的采集卡在 /dev/video0，输出 4K MJPEG 30fps
    std::string pipeline = 
        "v4l2src device=/dev/video0 ! "
        "image/jpeg, width=2560, height=1440, framerate=30/1 ! "
        "nvv4l2decoder mjpeg=1 ! "            // 硬件解码 MJPEG
        "nvvidconv ! "                         // 硬件颜色空间转换
        "video/x-raw, format=BGRx ! "          // 先转为 BGRx (NVIDIA 支持较好)
        "videoconvert ! "                      // 转为 OpenCV 标准 BGR
        "video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1";     // 只保留最新一帧，防止堆积延迟

    // 2. 打开视频流
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cerr << "错误: 无法使用 GStreamer 管道打开相机!" << std::endl;
        std::cerr << "请检查设备路径是否为 /dev/video0，或是否安装了 NVIDIA GStreamer 插件。" << std::endl;
        return -1;
    }

    std::cout << "成功打开相机。按 'q' 键退出。" << std::endl;

    // 3. 创建窗口（设置窗口属性以便可以缩放 4K 画面）
    cv::namedWindow("Xavier HW Decode", cv::WINDOW_NORMAL);
    cv::resizeWindow("Xavier HW Decode", 1280, 720); // 缩放到 720p 显示，避免撑满屏幕

    cv::Mat frame;
    while (true) {
        // 读取一帧
        if (!cap.read(frame)) {
            std::cerr << "错误: 无法接收帧。" << std::endl;
            break;
        }

        // 4. 显示画面
        cv::imshow("Xavier HW Decode", frame);

        // 按下 'q' 键退出
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();

    return 0;
}