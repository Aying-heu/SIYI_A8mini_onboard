#include "data_manager.hpp"
#include "data_types.hpp"

DataManager::DataManager(rclcpp::Node::SharedPtr node,const std::string& root_path_)
        : node_(node), clock_(node->get_clock()){
    data_config_.root_path_=root_path_;
    used_uav_history_.reserve(150);
    used_gimbal_history_.reserve(150);
    used_img_vector_.reserve(150);

    detected_target_msg_.reserve(MAX_TARGET_SIZE);
    filtered_target_msg_.reserve(MAX_TARGET_SIZE);
    data_config_.model_path_.reserve(2);
    data_config_.classes_.reserve(2);
    
    data_config_.cam_param_file_=data_config_.root_path_+"/A_params/camera_K.yaml";
    data_config_.zoom_correspond_file_=data_config_.root_path_+"/A_params/zoom_correspond.csv";
    data_config_.WorldPoints_correspind_file_=data_config_.root_path_+"/A_params/WorldPoints_BasedOn_camera_img.yaml";
    data_config_.model_path_.push_back(data_config_.root_path_+"/A_params/onnx_model/board.onnx");
    data_config_.model_path_.push_back(data_config_.root_path_+"/A_params/onnx_model/mark.onnx");
    data_config_.model_path_.push_back(data_config_.root_path_+"/A_params/onnx_model/Surf-CT-PatchTST-无动力20度基准-patchlen-30-stride-15-n_heads_ch-2-n_heads_tm-8-d_model-128-dff-256-nlayers-3-kernel-0-pred-5--展开给非线性.onnx");
    data_config_.classes_.push_back({"board"});
    data_config_.classes_.push_back({"mark_1","mark_A","mark_B","mark_C","mark_H","mark_X","mark_N","mark_O","mark_W"});

    // 创建主数据文件夹
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    std::string time_format = data_config_.root_path_ + "/data_%Y%m%d_%H%M%S";
    ss << std::put_time(std::localtime(&time_t), time_format.c_str());
    
    data_config_.base_dir_ = ss.str();
    data_config_.uav_state_file_=data_config_.base_dir_+"/uav_states.csv";
    data_config_.gimbal_state_file_=data_config_.base_dir_+"/gimbal_states.csv";
    data_config_.target_file_ori_=data_config_.base_dir_+"/target_states_ori.csv";
    data_config_.target_file_filtered_=data_config_.base_dir_+"/target_states_filtered.csv";
    data_config_.target_file_15hz_=data_config_.base_dir_+"/target_states_15hz.csv";
    data_config_.predictor_file_=data_config_.base_dir_+"/pred_result.bin";
    data_config_.image_dir_=data_config_.base_dir_+"/images/";

    std::filesystem::create_directories(data_config_.base_dir_);
    std::filesystem::create_directories(data_config_.image_dir_);
    std::filesystem::create_directories(data_config_.image_dir_+"/raw/");
    std::filesystem::create_directories(data_config_.image_dir_+"/detected/");
    std::filesystem::create_directories(data_config_.image_dir_+"/detected/ori");
    std::filesystem::create_directories(data_config_.image_dir_+"/detected/draw");
}

        
void DataManager::pushUavState(const UAV_Row& state){
    std::lock_guard<std::mutex> lock(uav_mtx_);
    uav_history_[state.nano_timestamp] = state;
    // 维护队列长度
    if (uav_history_.size() > MAX_UAV_SIZE) {
        auto oldest_it = uav_history_.begin();
        UAV_Row oldest_data = std::move(oldest_it->second);
        std::lock_guard<std::mutex> lock_used(used_uav_mtx_);
        used_uav_history_.emplace_back(std::move(oldest_data)); 
        uav_history_.erase(oldest_it);
    }
}

bool DataManager::getUavStateAt(uint64_t ts, UAV_Row& out_state){
    std::vector<UAV_Row> temp_move_data;

    {
        std::lock_guard<std::mutex> lock(uav_mtx_);
        if (uav_history_.empty()) return false;

        auto it_boundary = uav_history_.lower_bound(ts);
        if (it_boundary == uav_history_.begin()) return false;
        auto it_closest = std::prev(it_boundary);
        out_state = it_closest->second; 
    
        for (auto it = uav_history_.begin(); it != it_boundary; ++it) {
            // 使用 std::move 进一步压榨性能
            temp_move_data.push_back(std::move(it->second));
        }
        uav_history_.erase(uav_history_.begin(), it_boundary);
    }
    {
        std::lock_guard<std::mutex> lock_used(used_uav_mtx_);
        used_uav_history_.insert(used_uav_history_.end(), temp_move_data.begin(), temp_move_data.end());
    }
    return true;
}

bool DataManager::getLatestUavState(UAV_Row& out_state){
    std::lock_guard<std::mutex> lock(uav_mtx_);
    if (uav_history_.empty()) return false;
    out_state = uav_history_.rbegin()->second; 
    return true;
}

/*
********************************************************************
********************************************************************
********************************************************************
*/

void DataManager::pushGimbalState(const Gimbal_Row& state){
    std::lock_guard<std::mutex> lock(gimbal_mtx_);
    gimbal_history_[state.nano_timestamp] = state;
    if (gimbal_history_.size() > MAX_GIMBAL_SIZE) {
        auto oldest_it = gimbal_history_.begin();
        Gimbal_Row oldest_data = std::move(oldest_it->second);
        std::lock_guard<std::mutex> lock_used(used_gimbal_mtx_);
        used_gimbal_history_.emplace_back(std::move(oldest_data)); 
        gimbal_history_.erase(oldest_it);
    }
}

bool DataManager::getGimbalStateAt(uint64_t ts, Gimbal_Row& out_state){
    std::vector<Gimbal_Row> temp_move_data;

    {
        std::lock_guard<std::mutex> lock(gimbal_mtx_);
        if (gimbal_history_.empty()) return false;

        auto it_boundary = gimbal_history_.lower_bound(ts);
        if (it_boundary == gimbal_history_.begin()) return false;
        auto it_closest = std::prev(it_boundary);
        out_state = it_closest->second; 

        for (auto it = gimbal_history_.begin(); it != it_boundary; ++it) {
            // 使用 std::move 进一步压榨性能
            temp_move_data.push_back(std::move(it->second));
        }
        gimbal_history_.erase(gimbal_history_.begin(), it_boundary);
    }
    {
        std::lock_guard<std::mutex> lock_used(used_gimbal_mtx_);
        used_gimbal_history_.insert(used_gimbal_history_.end(), temp_move_data.begin(), temp_move_data.end());
    }
    
    return true;
}

bool DataManager::getLatestGimbalState(Gimbal_Row& out_state){
    std::lock_guard<std::mutex> lock(gimbal_mtx_);
    if (gimbal_history_.empty()) return false;
    out_state = gimbal_history_.rbegin()->second; 
    return true;
}

/*
********************************************************************
********************************************************************
********************************************************************
*/

void DataManager::pushRawImage(const cv::Mat& img, uint64_t ts){
    {
        std::lock_guard<std::mutex> lock(img_mtx_);
        Img_Row row;
        row.img=img;
        row.nano_timestamp=ts;
        raw_img_deque_.push_back(row);
        if (raw_img_deque_.size() > MAX_IMG_SIZE) {
            pushDetectedImage(raw_img_deque_.front());
            raw_img_deque_.pop_front();
        }
    }
    img_cv_.notify_one(); // 通知视觉线程：有新图来了！
}

bool DataManager::waitForImage(Img_Row& out_row){
    std::unique_lock<std::mutex> lock(img_mtx_);
    if (!img_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]{ return !raw_img_deque_.empty(); })) {
        return false; 
    }

    // 1. 取出最新的一帧
    out_row = std::move(raw_img_deque_.back());
    raw_img_deque_.pop_back();

    // 2. 【核心修改】处理掉被跳过的旧帧
    // 如果你希望保存这些没被检测的图（用于离线分析）：
    while(!raw_img_deque_.empty()){
        pushDetectedImage(std::move(raw_img_deque_.front()));
        raw_img_deque_.pop_front();
    }
    
    // 如果你根本不想要这些图，直接 raw_img_deque_.clear() 即可
    return true;
}

void DataManager::pushDetectedImage(const Img_Row& row){
    {
        std::lock_guard<std::mutex> lock(used_img_mtx_);
        used_img_vector_.emplace_back(row);
    }
}

/*
********************************************************************
********************************************************************
********************************************************************
*/

void DataManager::pushTargetMsg(prometheus_msgs::msg::Target target_msg_ori,prometheus_msgs::msg::Target target_msg_filtered){
    {
        std::lock_guard<std::mutex> lock(target_msg__mtx_);
        // 这里使用 std::move，会将传入的临时副本内容“移动”到 vector 中
        detected_target_msg_.emplace_back(std::move(target_msg_ori));
        if (detected_target_msg_.size() > MAX_TARGET_SIZE) {
            detected_target_msg_.erase(detected_target_msg_.begin());
        }
    }
    {
        std::lock_guard<std::mutex> lock(target_msg__mtx_);
        // 这里使用 std::move，会将传入的临时副本内容“移动”到 vector 中
        filtered_target_msg_.emplace_back(std::move(target_msg_filtered));
        if (filtered_target_msg_.size() > MAX_TARGET_SIZE) {
            filtered_target_msg_.erase(filtered_target_msg_.begin());
        }
    }
}

void DataManager::pushSample15hzTargetstate(TargetState target_state_){
    {
        std::lock_guard<std::mutex> lock(sample_15hz_target_mtx_);
        // 这里使用 std::move，会将传入的临时副本内容“移动”到 vector 中
        sample_15hz_target_state_.emplace_back(std::move(target_state_));
        if (sample_15hz_target_state_.size() > MAX_TARGET_SIZE) {
            sample_15hz_target_state_.erase(sample_15hz_target_state_.begin());
        }
    }
}

bool DataManager::getLatestTargetMsg(prometheus_msgs::msg::Target& target_msg){
    std::lock_guard<std::mutex> lock(target_msg__mtx_);
    if (detected_target_msg_.empty()) return false;
    const auto& latest_target = *detected_target_msg_.rbegin();
    target_msg = latest_target;
    return true;
}

/*
********************************************************************
********************************************************************
********************************************************************
*/

void DataManager::updateTargetStateFilterParameters(uint64_t t_ns, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat_meas) {
    std::lock_guard<std::mutex> lock(target_state_filter_mtx_);
    target_state_filter_.update_measurement(t_ns, pos, quat_meas);
}
TargetState DataManager::get_target_filtered_state(uint64_t query_time_ns){
    std::lock_guard<std::mutex> lock(target_state_filter_mtx_);
    return target_state_filter_.get_predicted_state(query_time_ns);
}
void DataManager::popUsedData(
        std::vector<UAV_Row>& uav_list, 
        std::vector<Gimbal_Row>& gimbal_list,
        std::vector<Img_Row>& img_list,
        std::vector<prometheus_msgs::msg::Target>& target_list_ori,
        std::vector<prometheus_msgs::msg::Target>& target_list_filtered,
        std::vector<TargetState>& target_list_15hz) {
    {
        std::lock_guard<std::mutex> lock(used_uav_mtx_);
        if(used_uav_history_.size()>1){
            UAV_Row last_element = std::move(used_uav_history_.back());
            uav_list.swap(used_uav_history_);
            used_uav_history_.emplace_back(last_element);
            uav_list.pop_back();
        }
    }
    {
        std::lock_guard<std::mutex> lock(used_gimbal_mtx_);
        if(used_gimbal_history_.size()>1){
            Gimbal_Row last_element = std::move(used_gimbal_history_.back());
            gimbal_list.swap(used_gimbal_history_);
            used_gimbal_history_.emplace_back(last_element);
            gimbal_list.pop_back();
        }
    }
    {
        std::lock_guard<std::mutex> lock(target_msg__mtx_);
        if(detected_target_msg_.size()>1){
            prometheus_msgs::msg::Target last_element = std::move(detected_target_msg_.back());
            target_list_ori.swap(detected_target_msg_);
            detected_target_msg_.emplace_back(last_element);
            target_list_ori.pop_back();
        }
        if(filtered_target_msg_.size()>1){
            prometheus_msgs::msg::Target last_element = std::move(filtered_target_msg_.back());
            target_list_filtered.swap(filtered_target_msg_);
            filtered_target_msg_.emplace_back(last_element);
            target_list_filtered.pop_back();
        }
    }
    {
        std::lock_guard<std::mutex> lock(sample_15hz_target_mtx_);
        if(sample_15hz_target_state_.size()>1){
            TargetState last_element = std::move(sample_15hz_target_state_.back());
            target_list_15hz.swap(sample_15hz_target_state_);
            sample_15hz_target_state_.emplace_back(last_element);
            target_list_15hz.pop_back();
        }
    }
    {
        std::lock_guard<std::mutex> lock(used_img_mtx_);
        img_list.swap(used_img_vector_);
    }
}