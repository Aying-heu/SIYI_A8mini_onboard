#include "camera_module.hpp"

CameraModule::CameraModule(std::shared_ptr<DataManager> data_manager, int device_id)
        :data_manager_(data_manager),device_id_(device_id){
    // cap_ = cv::VideoCapture(1);
    // std::string pipeline = "v4l2src device=/dev/video0 ! image/jpeg, width=3840, height=2160, framerate=30/1 ! nvv4l2decoder mjpeg=1 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
    // cv::VideoCapture cap_(pipeline, cv::CAP_GSTREAMER);

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
    cap_ = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
        std::cerr << "Error: Cannot open camera (camera )!" << std::endl;
        return;
    }
}

CameraModule::~CameraModule(){
    running_ = false;
    if (camera_thread_.joinable()) camera_thread_.join();
}

bool CameraModule::start(){
    if (running_) {
        return false;
    }
    running_ = true;
    camera_thread_ = std::thread(&CameraModule::run, this);
    return true;
}


void CameraModule::run() {
    cv::Mat frame;
    // cv::namedWindow("Xavier HW Decode", cv::WINDOW_NORMAL);
    // cv::resizeWindow("Xavier HW Decode", 1280, 720);
    while (running_.load()) {  
        if (cap_.read(frame)) {
            // 拿到图的第一时间打上 DataManager 的统一时间戳
            uint64_t ts = data_manager_->now_ns();
            // 直接推入原始队列
            const int target_width = 1280;
            double scale = static_cast<double>(target_width) / frame.cols; // 缩放比例（2560→1280 则 scale=0.5）
            int target_height = static_cast<int>(frame.rows * scale);
            cv::resize(frame, frame, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
            data_manager_->pushRawImage(frame.clone(), ts-10000000);
            // // 4. 显示画面
            // cv::imshow("Xavier HW Decode", frame);

            // // 按下 'q' 键退出
            // if (cv::waitKey(1) == 'q') {
            //     break;
            // }
        }
        // 不需要 sleep 太久，让它尽可能快地刷新硬件缓冲区
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


// 1768628789565609120

// 1768628278911676288