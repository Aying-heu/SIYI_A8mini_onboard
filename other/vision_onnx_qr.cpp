// vision_module.cpp
#include "vision_onnx_qr.hpp"
#include "data_manager.hpp" // 在 .cpp 里才真正包含 DataManager 的细节
#include <zbar.h> 

// 初始化静态成员（必须在类外定义一次）
Ort::Env VisionModule::env(ORT_LOGGING_LEVEL_WARNING, "VisionSystem");

VisionModule::VisionModule(std::shared_ptr<DataManager> data_manager) 
        : data_manager_(data_manager), running_(false) {

    load_model(data_manager_->get_config()->model_path_[0],
                board_model_,
                data_manager_->get_config()->classes_[0]);
    if(get_camera_K(data_manager_->get_config()->cam_param_file_,camera_K_))std::cout<<"相机内参数据读入成功成功"<<std::endl;
    if(get_zoom_correspond(data_manager_->get_config()->zoom_correspond_file_,zoom_correspond_))std::cout<<"放大倍率对应关系读取成功"<<std::endl;
    // if(get_WorldPoints_BasedOn_camera_img(data_manager_->get_config()->WorldPoints_correspind_file_,worldPoints_))std::cout<<"世界坐标系与相机对应关系读入成功"<<std::endl;

    if(get_WorldPoints_BasedOn_camera_img(data_manager_->get_config()->WorldPoints_correspind_file_,worldPoints_)) {
        std::cout<<"世界坐标系与相机对应关系读入成功"<<std::endl;
        // 定义偏移量（单位：米）
        float offset_x = -0.25f;  // 向左偏移25cm（X轴负方向）
        float offset_y = -0.48f;   // 向上偏移48cm（Y轴正方向）
        // 对所有世界坐标点进行偏移
        for(auto& pair : worldPoints_) {
            for(auto& point : pair.second) {
                point.x += offset_x;  // X坐标偏移
                point.y += offset_y;  // Y坐标偏移
                // Z坐标保持不变（平面上的点）
            }
        }
    }

    auto node = data_manager_->node();
    target_pub_ = node->create_publisher<prometheus_msgs::msg::Target>("/target1/prometheus/target_detection", 10);
    target_array_pub_ = node->create_publisher<prometheus_msgs::msg::TargetArray>("/target1/prometheus/target_state", 10);

    if (!target_pub_ || !target_array_pub_) {
        RCLCPP_FATAL(node->get_logger(), "Failed to create target publisher!");
    } else {
        RCLCPP_INFO(node->get_logger(), "target publisher created successfully");
    }
    
}

VisionModule::~VisionModule(){
    running_ = false;
    if (vision_thread_.joinable()) vision_thread_.join();
}

bool VisionModule::start_process(){
    if (running_) {
        return false;
    }
    running_ = true;
    vision_thread_ = std::thread(&VisionModule::process_loop, this);
    return true;
}

void VisionModule::process_loop() {
    // zbar::ImageScanner scanner;
    // scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    // zbar_image_scanner_set_config(scanner, zbar::ZBAR_DATABAR_EXP, zbar::ZBAR_CFG_ENABLE, 0);

    zbar::ImageScanner scanner;

    // 1. 禁用所有协议（这能关掉报错的 DataBar，还能大幅降低 CPU 占用，因为它不再盲目搜索）
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);

    // 2. 只开启二维码识别
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    // // 3. (可选) 如果你还想更彻底一点，显式关闭那几个容易报错的
    // scanner.set_config(zbar::ZBAR_DATABAR, zbar::ZBAR_CFG_ENABLE, 0);
    // scanner.set_config(zbar::ZBAR_DATABAR_EXP, zbar::ZBAR_CFG_ENABLE, 0);

    int has_idx=0;
    while (running_) {
        Img_Row row;
        // 1. 等待新图（这里会自动挂起，不占CPU）
        if (!data_manager_->waitForImage(row)) {
            continue; 
        }


        uint64_t target_ts = row.nano_timestamp; 
        UAV_Row uav_state;
        Gimbal_Row gimbal_state;
        bool has_uav_state = data_manager_->getUavStateAt(target_ts, uav_state);
        bool has_gimbal_state = data_manager_->getGimbalStateAt(target_ts, gimbal_state);

        if (!(has_uav_state && has_gimbal_state)) {
            has_idx+=1;
            if(has_idx%10==0){
                if (!has_uav_state && !has_gimbal_state) {
                    // 两者都找不到
                    std::cout << "Error: couldn't find closest uav_state AND gimbal_state at target_ts: " << target_ts << std::endl;
                } else if (!has_uav_state) {
                    // 仅无人机状态找不到
                    std::cout << "Error: couldn't find closest uav_state at target_ts: " << target_ts << std::endl;
                } else {
                    // 仅云台状态找不到
                    std::cout << "Error: couldn't find closest gimbal_state at target_ts: " << target_ts << std::endl;
                }
            }
            row.is_detected=false;
            data_manager_->pushDetectedImage(row);
            continue; // 保持原逻辑，跳过后续处理
        }

        // 2. 运行 ONNX 检测 (板子)
        float img_width=row.img.cols;
        float img_height=row.img.rows;
        std::vector<cv::Rect> boxes1, boxes2;
        std::vector<int> indexes1, indexes2;
        std::vector<std::string> conf_cls_names1, conf_cls_names2;
        std::vector<cv::Point3f> object_points;
        std::vector<cv::Point3f> object_points1;
        std::vector<cv::Point3f> object_points2;
        std::vector<cv::Point2f> image_points;
        float zoom = gimbal_state.zoom;
        float actual_zoom = zoom_correspond_[std::round(zoom * 10.0f) / 10.0f];

        Z_ONNX(row.img, board_model_, boxes1, indexes1, conf_cls_names1);
        for (int idx1 : indexes1) {
            cv::Rect board_rect = boxes1[idx1];
            board_rect = board_rect & cv::Rect(0, 0, img_width, img_height);
            if (board_rect.width <= 0 || board_rect.height <= 0) continue;
            
            cv::Mat roi = row.img(board_rect);
            cv::Mat gray;
            cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
            zbar::Image zbar_image(gray.cols, gray.rows, "Y800", gray.data,
                     gray.cols * gray.rows);
            int num_detected = scanner.scan(zbar_image);
            if(num_detected>0){
                for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
                        symbol != zbar_image.symbol_end(); ++symbol) {
                    if (symbol->get_type() == zbar::ZBAR_QRCODE) {
                        std::string content = symbol->get_data();
                        std::vector<cv::Point2f> corners;
                        std::vector<cv::Point2f> draw_corners;
                        
                        // 获取角点
                        for (int i = 0; i < symbol->get_location_size(); i++) {
                            int x = symbol->get_location_x(i);
                            int y = symbol->get_location_y(i);
                            float cx = x + boxes1[idx1].x;
                            float cy = y + boxes1[idx1].y;
                            // 由于图像放大倍数，需要回归
                            float out_Cx = (cx - img_width / 2.0f) / actual_zoom + img_width / 2.0f;
                            float out_Cy = (cy - img_height / 2.0f) / actual_zoom + img_height / 2.0f;
                            // 将放大（或缩小）后的坐标回算到原始图像坐标系
                            draw_corners.push_back(cv::Point2f(cx, cy));
                            out_Cx = (out_Cx - img_width/2.0f) * 2.0f + 2560/2.0f;
                            out_Cy = (out_Cy - img_height/2.0f) * 2.0f + 1440/2.0f;
                            corners.push_back(cv::Point2f(out_Cx, out_Cy));
                        }
                        for (size_t j = 0; j < corners.size(); j++) {
                            cv::circle(row.img, draw_corners[j], 10, cv::Scalar(0, 0, 0), -1);
                            int next = (j + 1) % corners.size();
                            cv::line(row.img, draw_corners[j], draw_corners[next], cv::Scalar(0, 0, 0), 10);
                            cv::Point2f text_pos = draw_corners[j] + cv::Point2f(8, 20);
                            putText(
                                row.img,                    // 缁樺埗鐩爣鍥惧儚
                                std::to_string(j + 1),       // 鏂囨湰鍐呭锛坖浠?寮€濮嬶紝+1杞负1-4锛?                                
                                text_pos,               // 鏂囧瓧浣嶇疆锛堣鐐瑰亸绉诲悗锛?                                
                                cv::FONT_HERSHEY_SIMPLEX,   // 瀛椾綋锛堢畝娲佹棤琛嚎锛?                                
                                2,                    // 瀛椾綋澶у皬锛堝彲璋冩暣锛?                                
                                cv::Scalar(0, 255, 255),  // 鏂囧瓧棰滆壊
                                5                       // 鏂囧瓧绮楃粏锛堢矖浣擄紝閬垮厤妯＄硦锛?                            );
                            );
                        }
                        // 需要检查 worldPoints 是否存在并且角点数量一致
                        if (corners.size() >= 3) {
                            auto it_wp = worldPoints_.find(content);
                            if (it_wp == worldPoints_.end()) {
                                std::cerr << "Warning: worldPoints not found for content: " << content << std::endl;
                            } else {
                                // 要求世界点数量与检测到的角点数量一致，避免 solvePnP 时对应关系错误
                                if (it_wp->second.size() != corners.size()) {
                                    std::cerr << "Warning: worldPoints size (" << it_wp->second.size()
                                         << ") != detected corners size (" << corners.size()
                                         << ") for content: " << content << ", skipping." << std::endl;
                                } else {
                                    object_points.insert(object_points.end(), it_wp->second.begin(), it_wp->second.end());
                                    image_points.insert(image_points.end(), corners.begin(), corners.end());
                                }
                            }
                        }
                        // 注意：这里没有break，会检测所有QR码
                        std::cout<< content << "  ";
                    }
                }
                std::cout << std::endl;
            }
        }
        if (object_points.empty() || object_points.size()<4) {
            std::cout<<"couldn't find qr code"<<std::endl;
            row.is_detected=false;
            data_manager_->pushDetectedImage(row);
            continue;
        }

        cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
        cv::Mat rvec, tvec;
        try{
            bool success = solvePnP(object_points, image_points, camera_K_, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            if(!success) {
                std::cerr << "solvePnP failed (no exception), skip timestamp: " << row.nano_timestamp << std::endl;
                row.is_detected=false;
                data_manager_->pushDetectedImage(row);
                continue;
            }

            const double rad2deg = 180.0 / M_PI;
            const double deg2rad = M_PI / 180.0;
            
            // 1. 将旋转向量转换为旋转矩阵
            cv::Mat R_rvec_target2cam;
            Rodrigues(rvec, R_rvec_target2cam);
            double tx = tvec.at<double>(0, 0);  // 第0行第0列（tx）
            double ty = tvec.at<double>(1, 0);  // 第1行第0列（ty）
            double tz = tvec.at<double>(2, 0);  // 第2行第0列（tz）
            Eigen::Vector3d target2cam(tx, ty, tz);
            Eigen::Matrix3d R_target2cam;
            {
                Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> map(R_rvec_target2cam.ptr<double>());
                R_target2cam = map;
            }

            // 2. 相机到无人机的固定安装偏移
            float cam_2_uav_x = 0.23;
            float cam_2_uav_y = 0;
            float cam_2_uav_z = -0.02;
            Eigen::Vector3d cam2uav(cam_2_uav_x, cam_2_uav_y, cam_2_uav_z);

            // 3. 相机到无人机的固定安装旋转（右下前 -> 前左上）
            // 相机坐标系：右x，下y，前z
            // 无人机坐标系：前x，左y，上z
            // 云台坐标系： 前x 左y 上z
            // 相机x轴（图像朝右）为无人机右（-y）
            // 相机y轴（图像超下）为无人机下（-z）
            // 相机z轴（相机视角）为无人机前（x）
            Eigen::Matrix3d R_cam2gimbal;
            R_cam2gimbal << 0, -1,  0,
                            0,  0, -1,
                            1,  0,  0;


            // 8. 获取无人机在世界坐标系中的位置和姿态
            // auto uav_q4_x = -uav_state->attitude_qx;
            // auto uav_q4_y = -uav_state->attitude_qy;
            // auto uav_q4_z = -uav_state->attitude_qz;
            // auto uav_q4_w = -uav_state->attitude_qw;
            double attitude_roll=uav_state.attitude_roll;
            double attitude_pitch=uav_state.attitude_pitch;
            double attitude_yaw=uav_state.attitude_yaw;
            Eigen::Matrix3d R_roll = Eigen::AngleAxisd(attitude_roll, Eigen::Vector3d::UnitX()).toRotationMatrix();   // 绕X轴滚转
            Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(attitude_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix(); // 绕Y轴俯仰
            Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(attitude_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();     // 绕Z轴偏航

            // 4. 构造无人机→世界的旋转矩阵（Eigen右乘逻辑：R = R_roll * R_pitch * R_yaw → 实际旋转顺序yaw→pitch→roll）
            Eigen::Matrix3d R_uav2world = R_roll * R_pitch * R_yaw;
            Eigen::Vector3d uav_position(uav_state.position_x, uav_state.position_y, uav_state.position_z);
            // Eigen::Quaterniond uav_attitude(uav_q4_w, uav_q4_x, uav_q4_y, uav_q4_z);
            // uav_attitude.normalize();
            // Eigen::Matrix3d R_uav2world = uav_attitude.toRotationMatrix();
            // Eigen::Vector3d euler_rad = R_uav2world.eulerAngles(2, 1, 0);  // 2=Z,1=Y,0=X → 顺序 Z-Y-X（yaw-pitch-roll）
            double roll_deg = attitude_roll*rad2deg;   // 滚转角（X轴）：对应 euler_rad[2]（因为顺序是 Z-Y-X，第三个元素是 X 轴旋转）
            double pitch_deg = attitude_pitch*rad2deg;  // 俯仰角（Y轴）：对应 euler_rad[1]
            double yaw_deg = attitude_yaw*rad2deg;    // 偏航角（Z轴）：对应 euler_rad[0]
            // roll_deg=pitch_180_to_0(roll_deg);
            // pitch_deg=pitch_180_to_0(pitch_deg);
            // roll_deg=pitch_180_to_0(roll_deg);
            // cout<<"无人机角度："<<roll_deg<<" "<<pitch_deg<<" "<<yaw_deg<<endl;
            
            // 4. 处理云台旋转（yaw相对无人机，pitch/roll相对地面）
            // 云台的yaw是相对于无人机的，需要取反（坐标系定义不同）
            double gimbal_yaw_rel = gimbal_state.yaw * deg2rad;
            
            // 云台的pitch是相对于水平面的，需要转换
            double gimbal_pitch_abs = gimbal_state.pitch* deg2rad;
            double gimbal_pitch_rel = gimbal_pitch_abs-pitch_deg* deg2rad;
            
            // 云台的roll是相对于垂直面的
            double gimbal_roll_abs = gimbal_state.roll * deg2rad;
            double gimbal_roll_rel = gimbal_roll_abs-roll_deg* deg2rad;

            // cout<<"云台相对无人机rad        "<<gimbal_roll_rel<<" "<<gimbal_pitch_rel<<" "<<gimbal_yaw_rel<<endl;
            
            // 5. 构建云台的旋转矩阵
            // 注意：云台旋转是应用在相机固定安装之后的
            // 旋转顺序：先yaw（绕Z轴，相对于无人机），然后pitch（绕Y轴，绝对），最后roll（绕X轴，绝对）
            Eigen::Matrix3d R_gimbal_yaw, R_gimbal_pitch, R_gimbal_roll;
            R_gimbal_yaw = Eigen::AngleAxisd(gimbal_yaw_rel, Eigen::Vector3d::UnitZ());
            R_gimbal_pitch = Eigen::AngleAxisd(gimbal_pitch_rel, Eigen::Vector3d::UnitY());
            R_gimbal_roll = Eigen::AngleAxisd(gimbal_roll_rel, Eigen::Vector3d::UnitX());
            
            // 完整的云台旋转矩阵    云台相对于无人机
            Eigen::Matrix3d R_gimbal = R_gimbal_roll * R_gimbal_pitch * R_gimbal_yaw;
            
            // 6. 完整的相机到无人机的旋转矩阵
            // 先应用固定安装旋转，再应用云台旋转
            Eigen::Matrix3d R_cam2uav =  R_gimbal *R_cam2gimbal.transpose();
            
            // 7. 将目标从相机坐标系转换到无人机坐标系
            Eigen::Matrix3d R_target2uav = R_cam2uav * R_target2cam;
            Eigen::Vector3d target2uav = R_cam2uav * target2cam + cam2uav;
        
            
            // 9. 将目标从无人机坐标系转换到世界坐标系
            Eigen::Vector3d target_in_world = R_uav2world * target2uav + uav_position;
            Eigen::Matrix3d R_target_in_world = R_uav2world * R_target2uav;
            
            // 10. 提取欧拉角（Z-Y-X顺序：yaw绕Z，pitch绕Y，roll绕X）
            double yaw, pitch, roll;
            double sy = sqrt(R_target_in_world(0,0)*R_target_in_world(0,0) + R_target_in_world(1,0)*R_target_in_world(1,0));
            
            if (sy > 1e-6) {  // 非万向锁
                yaw = atan2(R_target_in_world(1,0), R_target_in_world(0,0));
                pitch = atan2(-R_target_in_world(2,0), sy);
                roll = atan2(R_target_in_world(2,1), R_target_in_world(2,2));
            } else {  // 万向锁
                yaw = atan2(-R_target_in_world(0,1), R_target_in_world(1,1));
                pitch = atan2(-R_target_in_world(2,0), sy);
                roll = 0.0;
            }
            
            // 转换为角度
            Eigen::Vector3d target_rad;
            Eigen::Vector3d target_degree;
            target_rad[0] = static_cast<double>(yaw);    // yaw（Z轴）
            target_rad[1] = static_cast<double>(pitch);  // pitch（Y轴）
            target_rad[2] = static_cast<double>(roll);   // roll（X轴）
            target_degree[0] = static_cast<double>(yaw * rad2deg);    // yaw（Z轴）
            target_degree[1] = static_cast<double>(pitch * rad2deg);  // pitch（Y轴）
            target_degree[2] = static_cast<double>(roll * rad2deg);   // roll（X轴）
            
            // 从旋转矩阵构造四元数
            Eigen::Quaterniond target_quat(R_target_in_world);

            auto target_msg = std::make_unique<prometheus_msgs::msg::Target>(); 
            auto target_array_msg = std::make_unique<prometheus_msgs::msg::TargetArray>();    
            
            rclcpp::Time img_ros_time(target_ts);
            target_msg->timestamp.stamp = img_ros_time;
            target_msg->px=target_in_world[0];
            target_msg->py=target_in_world[1];
            target_msg->pz=target_in_world[2];
            target_msg->attitude[0]=target_rad[0];
            target_msg->attitude[1]=target_rad[1];
            target_msg->attitude[2]=target_rad[2];
            target_msg->attitude_q.x=target_quat.x();
            target_msg->attitude_q.y=target_quat.y();
            target_msg->attitude_q.z=target_quat.z();
            target_msg->attitude_q.w=target_quat.w();

            data_manager_->pushDetectedTargetMsg(*target_msg);                                                                                                                                   
            std::fill(target_array_msg->targets.begin(), target_array_msg->targets.end(), *target_msg);
            target_array_msg->timestamp.stamp = img_ros_time;
            // target_array_msg->targets.assign(50, *target_msg); // 自动调整大小并填充 50 个
            target_array_pub_->publish(std::move(target_array_msg));
            target_pub_->publish(std::move(target_msg));


        } catch (const cv::Exception& e) {
            std::cerr << "Caught cv::Exception at timestamp " << row.nano_timestamp << ": " << e.what() << std::endl;
            row.is_detected=false;
            data_manager_->pushDetectedImage(row);
            continue;
        }

        row.is_detected=true;
        if(row.is_detected){
            // 专门为显示创建一张小图
            static cv::Mat show_img; 
            cv::resize(row.img, show_img, cv::Size(1280, 720)); // 先缩小
            
            // 窗口初始化只做一次
            static bool window_inited = false;
            if(!window_inited){
                cv::namedWindow("detected", cv::WINDOW_AUTOSIZE);
                window_inited = true;
            }

            cv::imshow("detected", show_img); // 显示缩小后的图，极快
            cv::waitKey(1); 
        }

        // 4. 将检测完的图（带框）送入待保存队列
        data_manager_->pushDetectedImage(row);
    }
}

bool VisionModule::get_camera_K(const std::string& cam_param_path, cv::Mat& camera_K) {
    // 1. 打开YAML文件
    std::cout<<"读入内参1"<<std::endl;
    cv::FileStorage fs(cam_param_path, cv::FileStorage::READ);
    std::cout<<"读入内参2"<<std::endl;
    if (!fs.isOpened()) {
        std::cerr << "[ERROR] 无法打开YAML文件: " << cam_param_path << std::endl;
        return false;
    }

    // 2. 检查camera_K字段是否存在
    std::cout<<"读入内参3"<<std::endl;
    if (!fs["camera_K"].isNone()) {
        fs["camera_K"] >> camera_K;
    } else {
        std::cerr << "[ERROR] YAML文件中缺失 camera_K 字段！" << std::endl;
        fs.release(); // 这里主动释放是为了明确，实际析构也会释放
        return false;
    }
    std::cout<<"读入内参4"<<std::endl;

    // 3. 校验矩阵维度（必须是3x3）
    if (camera_K.rows != 3 || camera_K.cols != 3) {
        std::cerr << "[ERROR] camera_K 维度错误！期望3x3，实际: " 
                  << camera_K.rows << "x" << camera_K.cols << std::endl;
        return false;
    }

    // 4. 显式转换为double类型（确保精度，可选但推荐）
    camera_K.convertTo(camera_K, CV_64F);

    // 5. 打印内参（方便调试）
    std::cout << "[INFO] 读取的camera_K (3x3):\n" << camera_K << std::endl << std::endl;

    // 无需手动release，析构会自动处理
    return true;
}

bool VisionModule::get_WorldPoints_BasedOn_camera_img(const std::string& yaml_path, std::map<std::string, std::vector<cv::Point3f>>& worldPoints){
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "错误：OpenCV无法打开YAML文件！路径：" << yaml_path << std::endl;
        return false;
    }

    // 读取board参数（二维码收缩计算需要）
    cv::FileNode board_node = fs["board"];
    if (board_node.empty()) {
        std::cerr << "错误：YAML 缺失 'board' 节点！路径：" << yaml_path << std::endl;
        fs.release();
        return false;
    }
    int code_margin = (int)board_node["code_margin"];
    int number_of_pixels = (int)board_node["number_of_pixels"];

    // 工具：从 FileNode 中安全读取 id 为字符串（若为数字则转成字符串）
    auto fileNodeToIdStr = [](const cv::FileNode &n) -> std::string {
        if (n.empty()) return std::string();
        // 优先尝试以字符串读取
        std::string s = (std::string)n;
        if (!s.empty()) return s;
        // 若字符串为空，尝试读取为整数或实数并转为字符串
        try {
            int vi = (int)n;
            return std::to_string(vi);
        } catch (...) {
        }
        try {
            double vd = (double)n;
            int vi = static_cast<int>(vd);
            return std::to_string(vi);
        } catch (...) {
        }
        return std::string();
    };

    cv::FileNode special_node = fs["special_mark"];
    if (special_node.empty()) {
        std::cerr << "警告：YAML 中未找到 'special_mark' 节点（可选）。\n";
        return false;
    }
    for (auto it = special_node.begin(); it != special_node.end(); ++it) {
        cv::FileNode item = *it;
        std::string id = fileNodeToIdStr(item["id"]); // 转换id为字符串键
        float side_len = (float)item["side_length"];

        // 计算QR面中心点
        cv::FileNode qr_ref = item["QR_top_left_reference_point"];
        float qr_x = (float)qr_ref[0];
        float qr_y = (float)qr_ref[1];
        cv::Point3f qr_center(qr_x + side_len/2, qr_y + side_len/2, 0.0f);

        // 计算Aruco面中心点
        cv::FileNode aruco_ref = item["Aruco_top_left_reference_point"];
        float ar_x = (float)aruco_ref[0];
        float ar_y = (float)aruco_ref[1];
        cv::Point3f aruco_center(ar_x + side_len/2, ar_y + side_len/2, 0.0f);

        // 存入结果（第一个为QR中心，第二个为Aruco中心）
        worldPoints[id] = {qr_center, aruco_center};
    }

    // 处理QR_code_surface（计算收缩后的四个角点）
    cv::FileNode qr_node = fs["QR_code"];
    if (qr_node.empty()) {
        std::cerr << "警告：YAML 中未找到 'QR_code' 节点（可选）。\n";
        return false;
    }
    for (auto it = qr_node.begin(); it != qr_node.end(); ++it) {
        cv::FileNode item = *it;
        std::string id = fileNodeToIdStr(item["id"]);
        float side_len = (float)item["side_length"];
        cv::FileNode ref = item["top_left_reference_point"];
        float x = (float)ref[0];
        float y = (float)ref[1];

        // 计算收缩量（根据码边距和像素数）
        float pixel_size = side_len / (number_of_pixels + 2 * code_margin);
        float shrink = code_margin * pixel_size; // 单边收缩量
        float inner_side = side_len - 2 * shrink; // 收缩后边长
        float inner_x = x + shrink; // 收缩后左上角X
        float inner_y = y + shrink; // 收缩后左上角Y

        std::vector<cv::Point3f> corners;
        corners.emplace_back(inner_x, inner_y, 0.0f);
        corners.emplace_back(inner_x, inner_y + inner_side, 0.0f);
        corners.emplace_back(inner_x + inner_side, inner_y + inner_side, 0.0f);
        corners.emplace_back(inner_x + inner_side, inner_y, 0.0f);
        worldPoints[id] = corners;
    }

    // 处理Aruco_surface（计算原始四个角点）
    cv::FileNode aruco_node = fs["Aruco"];
    if (aruco_node.empty()) {
        std::cerr << "警告：YAML 中未找到 'Aruco' 节点（可选）。\n";
        return false;
    }
    for (auto it = aruco_node.begin(); it != aruco_node.end(); ++it) {
        cv::FileNode item = *it;
        std::string id = fileNodeToIdStr(item["id"]);
        float side_len = (float)item["side_length"];
        cv::FileNode ref = item["top_left_reference_point"];
        float x = (float)ref[0];
        float y = (float)ref[1];

        // 四个角点（顺时针：左上→右上→右下→左下）
        std::vector<cv::Point3f> corners;
        corners.emplace_back(x, y, 0.0f);
        corners.emplace_back(x + side_len, y, 0.0f);
        corners.emplace_back(x + side_len, y + side_len, 0.0f);
        corners.emplace_back(x, y + side_len, 0.0f);
        worldPoints[id] = corners;
    }

    fs.release();
    return true;
}
void VisionModule::load_model(
        const std::string& model_path, 
        ModelSession& model,
        const std::vector<std::string>& class_names){

    Ort::SessionOptions session_options;
    OrtCUDAProviderOptions cuda_options;

    cuda_options.device_id = 0;
    cuda_options.arena_extend_strategy = 0;
    cuda_options.gpu_mem_limit = static_cast<size_t>(6) * 1024 * 1024 * 1024; // 4GB
    cuda_options.do_copy_in_default_stream = 1;
    session_options.AppendExecutionProvider_CUDA(cuda_options);
    
    try {
        model.session = Ort::Session(env, model_path.c_str(), session_options);
        
        // 获取模型输入输出信息
        Ort::AllocatorWithDefaultOptions allocator;
        
        // 获取输入名称
        size_t num_input_nodes = model.session.GetInputCount();
        for (size_t i = 0; i < num_input_nodes; i++) {
            auto input_name = model.session.GetInputNameAllocated(i, allocator);
            std::string name_str(input_name.get());
            model.input_names_str.push_back(name_str);
            model.input_names.push_back(model.input_names_str.back().c_str());
        }
        
        // 获取输出名称
        size_t num_output_nodes = model.session.GetOutputCount();
        for (size_t i = 0; i < num_output_nodes; i++) {
            auto output_name = model.session.GetOutputNameAllocated(i, allocator);
            std::string name_str(output_name.get());
            model.output_names_str.push_back(name_str);
            model.output_names.push_back(model.output_names_str.back().c_str());
            std::cout<<name_str<<std::endl;
        }
        // 获取输入维度
        auto input_dims = model.session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        model.input_h = input_dims[2];
        model.input_w = input_dims[3];
        
        // 获取输出维度
        auto output_dims = model.session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        std::cout<<"output_dims.size()="<<output_dims.size()<<std::endl;       
        model.output_h = output_dims[1];
        model.output_w = output_dims[2];

        model.class_names=class_names;
        
        std::cout << "Model loaded: " << model_path << std::endl
                  << " Input: " << model.input_w << "x" << model.input_h << std::endl
                  << " Output: " << model.output_w << "x" << model.output_h << std::endl
                  <<"  and "<<output_dims[0]<< std::endl;
                  
         // 打印输入输出名称
        std::cout << "Input names: ";
        for (const auto& name : model.input_names) {
            std::cout << name << ", ";
        }
        std::cout << "\nOutput names: ";
        for (const auto& name : model.output_names) {
            std::cout << name << ", ";
        }
        std::cout << std::endl;
    } catch (const Ort::Exception& e) {
        std::cerr << "模型加载失败: " << e.what() << std::endl;
        exit(1);
    }
}

void VisionModule::Z_ONNX(
        cv::Mat& frame, 
        ModelSession& model, 
        std::vector<cv::Rect>& boxes, 
        std::vector<int>& indexes, 
        std::vector<std::string>& conf_cls_names){

    boxes.clear();
    indexes.clear();
    if (frame.empty()) {
        std::cerr << "Empty frame in Z_ONNX!" << std::endl;
        return;
    }
    
    // 重用内存
    cv::Mat resized;
    cv::cuda::GpuMat gpu_frame, gpu_resized;
    cv::cuda::GpuMat gpu_blob;
    
    // 上传到GPU
    gpu_frame.upload(frame);
    
    // GPU加速的resize
    cv::cuda::resize(gpu_frame, gpu_resized,cv::Size(model.input_w, model.input_h));
    gpu_resized.download(resized);

    // 创建ONNX输入张量（使用GPU内存）
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
    
    // 创建blob（确保每次都是新的）
    cv::Mat blob = cv::dnn::blobFromImage(resized, 1/255.0, cv::Size(), cv::Scalar(), true, false);
    
    // 创建ONNX输入张量
    std::vector<int64_t> input_shape = {1, 3, model.input_h, model.input_w};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, blob.ptr<float>(), blob.total(), 
        input_shape.data(), input_shape.size());
    
    // 异步推理
    Ort::RunOptions run_options;
    run_options.SetRunLogVerbosityLevel(ORT_LOGGING_LEVEL_WARNING);
    
    auto outputs = model.session.Run(run_options, 
        model.input_names.data(), &input_tensor, 1, 
        model.output_names.data(), 1);
        
    if (outputs.empty() || !outputs[0].IsTensor()) {
        std::cerr << "ONNX Runtime returned invalid output" << std::endl;
        return;
    }

    if (model.output_h <= 0 || model.output_w <= 0) {
        std::cerr << "Invalid output dimensions: " 
                  << model.output_h << "x" << model.output_w << std::endl;
        return;
    }
    
    const float* output_data = outputs[0].GetTensorData<float>();
    cv::Mat det_output(model.output_h, model.output_w, CV_32F, (float*)output_data);
    det_output = det_output.t();

    float scale_x = frame.cols / (float)model.input_w; // 宽度缩放比例
    float scale_y = frame.rows / (float)model.input_h; // 高度缩放比例

    const int num_classes = model.output_h - 4; // 类别数量
    const float conf_threshold = 0.25f;
    const float* data_ptr = (const float*)det_output.data; // 拿到原始数据首地址
    const int row_step = det_output.cols; // 每行有多少个元素

    std::vector<float> confidences; // 置信度

    confidences.reserve(100); 
    boxes.reserve(100);
    conf_cls_names.reserve(100);

    for (int i = 0; i < det_output.rows; i++) { // 遍历输出    2100个候选框？

        // // 提取第i行、从第4列到最后一列的子矩阵
        // cv::Mat row_sub = det_output.row(i).colRange(4, model.output_h);
        // // 找子矩阵的最大值和对应位置（col是相对子矩阵的列索引，需+4还原全局列索引）
        // double max_val;
        // cv::Point max_loc;
        // cv::minMaxLoc(row_sub, nullptr, &max_val, nullptr, &max_loc);

        // // 赋值给conf和nc
        // float conf = static_cast<float>(max_val);
        // int nc = 4 + max_loc.x; // max_loc.x是子矩阵的列索引，+4还原全局列索引（4开始）

        // if(conf<=0.25)continue;
        // float cx = det_output.at<float>(i, 0);  // 中心点x坐标
        // float cy = det_output.at<float>(i, 1);  // 中心点y坐标
        // float ow = det_output.at<float>(i, 2);  // 宽度
        // float oh = det_output.at<float>(i, 3);  // 高度
        // int x = static_cast<int>((cx - 0.5 * ow) * scale_x);    // 左上角x坐标
        // int y = static_cast<int>((cy - 0.5 * oh) * scale_y);    // 左上角y坐标
        // int width = static_cast<int>(ow * scale_x);     // 宽度
        // int height = static_cast<int>(oh * scale_y);    // 高度
        // boxes.emplace_back(x, y, width, height);        // 添加检测框
        // confidences.push_back(conf);                    // 添加置信度
        // conf_cls_names.push_back(model.class_names[nc-4]);

        // double max_val;
        // cv::Point max_loc;
        // cv::minMaxLoc(det_output.row(i).colRange(4, model.output_h), nullptr, &max_val, nullptr, &max_loc);

        // if(static_cast<float>(max_val)<=0.25)continue;

        // boxes.emplace_back(static_cast<int>((det_output.at<float>(i, 0) - 0.5 * det_output.at<float>(i, 2)) * scale_x),
        //                     static_cast<int>((det_output.at<float>(i, 1) - 0.5 * det_output.at<float>(i, 3)) * scale_y),
        //                     static_cast<int>(det_output.at<float>(i, 2) * scale_x), 
        //                     static_cast<int>(det_output.at<float>(i, 3) * scale_y));        // 添加检测框
        // confidences.push_back(static_cast<float>(max_val));                    // 添加置信度
        // conf_cls_names.push_back(model.class_names[max_loc.x]);

        const float* row = data_ptr + (i * row_step);
        float max_score = row[4];
        int class_id = 0;
        for (int j = 1; j < num_classes; j++) {
            if (row[4 + j] > max_score) {
                max_score = row[4 + j];
                class_id = j;
            }
        }
        if (max_score <= conf_threshold) continue;
        float cx = row[0];
        float cy = row[1];
        float w  = row[2];
        float h  = row[3];

        int left   = static_cast<int>((cx - 0.5f * w) * scale_x);
        int top    = static_cast<int>((cy - 0.5f * h) * scale_y);
        int width  = static_cast<int>(w * scale_x);
        int height = static_cast<int>(h * scale_y);

        boxes.emplace_back(left, top, width, height);
        confidences.push_back(max_score);
        conf_cls_names.push_back(model.class_names[class_id]);
    }

    // 两个数字，置信度小于前者的过滤，重叠范围大于后者的抑制    indexes：最终保留的检测框索引
    cv::dnn::NMSBoxes(boxes, confidences, 0.25, 0.45, indexes); // 非极大值抑制

    for (int idx : indexes) {       // 遍历索引
        cv::rectangle(frame, boxes[idx], cv::Scalar(0, 255, 0), 2); // 绘制检测框
        std::string label = cv::format("%s: %.2f", conf_cls_names[idx].c_str(), confidences[idx]);
        cv::putText(frame, label, cv::Point(boxes[idx].x, boxes[idx].y - 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 0.8); // 绘制标签
    }

    for (auto& box : boxes) {
        box = box & cv::Rect(0, 0, frame.cols, frame.rows);
    }
}


bool VisionModule::get_zoom_correspond(const std::string& file_path, 
                                std::map<float, float>& corres) {
    std::ifstream file(file_path);
    if (!file) return false;
    
    std::string line;
    std::getline(file, line); // 跳过表头
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::istringstream ss(line);
        std::string target_str, actual_str;
        
        if (std::getline(ss, target_str, ',') && 
            std::getline(ss, actual_str, ',')) {
            try {
                float target = std::stof(target_str);
                float actual = std::stof(actual_str);
                
                // 四舍五入到一位小数
                target = std::round(target * 10.0f) / 10.0f;
                
                corres[target] = actual;
            } catch (...) {
                // 忽略转换错误
            }
        }
    }
    corres[1.0f] = 1.0f;
    return !corres.empty();
}