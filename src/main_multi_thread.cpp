#include "data_types.hpp"

#include "data_manager.hpp"
#include "data_logger.hpp"

#include "camera_module.hpp"
#include "gimbal_module.hpp"
#include "vision_onnx_aruco.hpp"
// #include "vision_onnx_qr.hpp"
// #include "vision_onnx_onnx.hpp"

#include "uav_module.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("uav_vision_system");
    auto data_manager = std::make_shared<DataManager>(node,"/home/bsa/A_vision_relate/");
    std::cout<<"data_manager started successful"<<std::endl;
        

    auto data_logger = std::make_shared<DataLogger>(data_manager);
    data_logger->start();
    std::cout<<"data_logger started successful"<<std::endl;


    auto cam_module = std::make_shared<CameraModule>(data_manager, 0);
    if(!cam_module->start()){
        std::cout << "相机开启失败"<<std::endl;
        return 0;
    }
    std::cout<<"cam_module started successful"<<std::endl;


    auto gimbal = std::make_shared<GimbalModule>(data_manager);
    gimbal->start();
    std::cout<<"gimbal started successful"<<std::endl;


    auto uav = std::make_shared<UavModule>(data_manager);
    std::cout<<"uav started successful"<<std::endl;


    auto vision_module = std::make_shared<VisionModule>(data_manager);
    if(!vision_module ->start_process()){
        std::cout << "检测线程开启失败"<<std::endl;
        return 0;
    }
    std::cout<<"vision_module started successful"<<std::endl;

    
    // 3. 启动 ROS 订阅 (主线程 spin)
    rclcpp::spin(uav); 

    rclcpp::shutdown();
    return 0;
}