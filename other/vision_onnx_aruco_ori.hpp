#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/cudawarping.hpp>  // 必须包含这个，cv::cuda::resize 才能找到
#include <opencv2/cudaimgproc.hpp> // 如果你用到了 cv::cuda::cvtColor，也需要这个

  

#include <opencv2/core/cuda.hpp>
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <string>
#include <thread>
#include <fstream> 
#include <opencv2/aruco.hpp> 
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <prometheus_msgs/msg/target.hpp>
#include <prometheus_msgs/msg/target_array.hpp>
#include "data_types.hpp"

// 前向声明：告诉编译器 DataManager 是个类，但先不用包含它的细节
class DataManager;


// 模型会话结构体：建议放在单独的命名空间或类内
struct ModelSession {
    Ort::Session session{nullptr};
    int input_w = 0;
    int input_h = 0;
    int output_h = 0;
    int output_w = 0;
    std::vector<std::string> input_names_str;
    std::vector<std::string> output_names_str;
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    std::vector<std::string> class_names;
};

class VisionModule {
public:
    // 构造函数传入 DataManager 指针
    VisionModule(std::shared_ptr<DataManager> data_manager);
    ~VisionModule();

    // 启动处理线程的接口
    bool start_process();
private:
    double distance_=-1.0;
    // 静态成员变量：整个程序共享一个 ONNX 环境
    static Ort::Env env;
    std::shared_ptr<DataManager> data_manager_;

    // 模型实例
    ModelSession board_model_;
    ModelSession mark_model_;

    cv::Mat camera_K_;
    std::map<std::string, std::vector<cv::Point3f>> worldPoints_;
    std::map<float,float>zoom_correspond_;

    rclcpp::Publisher<prometheus_msgs::msg::Target>::SharedPtr target_pub_;
    rclcpp::Publisher<prometheus_msgs::msg::TargetArray>::SharedPtr target_array_pub_;

    // 内部函数：不对外公开
    void load_model(const std::string& model_path, ModelSession& model, const std::vector<std::string>& class_names);
    bool get_camera_K(const std::string& cam_param_path, cv::Mat& camera_K);
    bool get_WorldPoints_BasedOn_camera_img(const std::string& yaml_path, std::map<std::string, std::vector<cv::Point3f>>& worldPoints);
    bool get_zoom_correspond(const std::string& file_path, std::map<float, float>& corres);
    void Z_ONNX(cv::Mat& frame, ModelSession& model, 
                std::vector<cv::Rect>& boxes, 
                std::vector<int>& indexes, 
                std::vector<std::string>& conf_cls_names);

    void onnx_aruco(
            cv::Mat& frame,
            UAV_Row& uav_state, Gimbal_Row& gimbal_state,
            std::vector<cv::Point3f>& object_points,std::vector<cv::Point2f>& image_points);
    void onnx_color_corners(
            cv::Mat& frame,
            UAV_Row& uav_state, Gimbal_Row& gimbal_state,
            std::vector<cv::Point3f>& object_points,std::vector<cv::Point2f>& image_points);

    // 线程主循环函数
    void process_loop();
    bool running_;
    std::thread vision_thread_;


    cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::Ptr<cv::aruco::Dictionary>(
        new cv::aruco::Dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000))
    );
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params =
        cv::Ptr<cv::aruco::DetectorParameters>(new cv::aruco::DetectorParameters());


    std::vector<cv::Point2f> order_points(std::vector<cv::Point2f> pts);
    std::vector<cv::Point2f> get_perfect_four_corners(const cv::Mat& mask);
    std::vector<cv::Point2f> reorder_by_marker_A(
        const std::vector<cv::Point2f>& geometric_pts, 
        const cv::Point2f& marker_a_center) ;
};