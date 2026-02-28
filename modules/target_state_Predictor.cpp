#include "target_state_Predictor.hpp"
#include "data_manager.hpp" 
#include <chrono>
#include <cmath>
#include <filesystem> // C++17

TargetStatePredictor::TargetStatePredictor(std::shared_ptr<DataManager> data_manager) 
    : data_manager_(data_manager) 
{
    // 初始化 ONNX Env (日志级别 warning)
    ort_env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "TargetPredictor");

    auto node = data_manager_->node();
    target_array_pub_ = node->create_publisher<prometheus_msgs::msg::TargetArray>("/target1/prometheus/target_state", 10);
    
    if (!target_array_pub_) {
        RCLCPP_FATAL(node->get_logger(), "Failed to create target publisher!");
    }

    // --- 加载模型 ---
    // 假设配置路径存在于 data_manager->get_config()->model_path_[0]
    // 这里你需要根据实际 config 结构修改获取路径的方式
    // std::string model_path = "/home/robot/AAA/PatchTST-固定模型2/Z_Onnx/runs/weights/onnx_1.onnx"; 
    std::string model_path = data_manager_->get_config()->model_path_[2];
    load_model(model_path);
}

TargetStatePredictor::~TargetStatePredictor() {
    stop();
}

bool TargetStatePredictor::start() {
    if (is_running_) return true;
    is_running_ = true;
    predictor_thread_ = std::thread(&TargetStatePredictor::run, this);
    return true;
}

void TargetStatePredictor::stop() {
    is_running_ = false;
    if (predictor_thread_.joinable()) {
        predictor_thread_.join();
    }
}

// 封装单次采样逻辑，避免代码重复
void TargetStatePredictor::process_single_sample(uint64_t timestamp_ns) {
    // 获取指定时刻的状态
    TargetState current_state = data_manager_->get_target_filtered_state(timestamp_ns);
    if (current_state.timestamp == 0) {
        // 如果 DataManager 没找到数据返回了空结构体，这里直接丢弃，不要存入 history
        // std::cerr << "Warning: Got invalid state with ts=0 at " << timestamp_ns << std::endl;
        return; 
    }
    data_manager_->pushSample15hzTargetstate(current_state);
    {
        std::lock_guard<std::mutex> lock(buffer_mtx_);
        history_buffer_.push_back(current_state);
        if (history_buffer_.size() > max_history_size_) {
            history_buffer_.pop_front();
        }
    }
}



// --- 核心逻辑封装 ---
// 这个函数是“无状态”于线程的，它只依赖传入的时间戳
void TargetStatePredictor::tick(uint64_t now_ns) {
    // 首次运行时初始化时间基准
    if (next_sample_time_ns_ == 0) {
        next_sample_time_ns_ = now_ns;
        last_sync_time_ns_ = now_ns;
        last_predict_time_ns_ = now_ns;
        return; // 初始化后直接返回，等待下一次 tick
    }

    // ---------------------------------------------------------
    // 逻辑A: 5秒同步 (使用传入的 now_ns)
    // ---------------------------------------------------------
    if (now_ns - last_sync_time_ns_ >= sync_interval_ns_) {
        // ... (同之前的逻辑，把 ts 换成 now_ns) ...
        if (next_sample_time_ns_ < now_ns) {
            uint64_t time_diff = now_ns - next_sample_time_ns_;
            if (time_diff > sample_interval_ns_) {
                 // 补齐逻辑
                 while (next_sample_time_ns_ < now_ns) {
                    process_single_sample(next_sample_time_ns_);
                    next_sample_time_ns_ += sample_interval_ns_;
                }
            }
        }
        last_sync_time_ns_ = now_ns;
    }

    // ---------------------------------------------------------
    // 逻辑B: 15Hz 采样
    // ---------------------------------------------------------
    // 离线模式下，如果 Log 两次 tick 间隔很大，这里会自动循环多次以补齐
    while (now_ns >= next_sample_time_ns_) {
        process_single_sample(next_sample_time_ns_);
        next_sample_time_ns_ += sample_interval_ns_;
    }

    // ---------------------------------------------------------
    // 逻辑C: 预测 (0.5s)
    // ---------------------------------------------------------
    if (now_ns - last_predict_time_ns_ >= predict_interval_ns_) {
        std::deque<TargetState> current_history;
        {
            std::lock_guard<std::mutex> lock(buffer_mtx_);
            // 复制一份，避免长时间锁住 buffer
            if (history_buffer_.size() >= max_history_size_) {
                current_history = history_buffer_;
            }
        }

        if (!current_history.empty()) {
            // 1. 执行预测 (内部已包含了 save_to_binary)
            std::vector<TargetState> future_states = predict_future(current_history);
            
            // 2. 发布 ROS 消息
            if (!future_states.empty()) {
                auto target_array_msg = std::make_unique<prometheus_msgs::msg::TargetArray>();
                
                // 填充 Header
                // target_array_msg->header.stamp = rclcpp::Time(static_cast<int64_t>(now_ns));

                size_t array_capacity = target_array_msg->targets.size(); 

                // 2. 循环填充 TargetArray
                // 限制最大取 50 个，防止溢出，同时兼容模型输出较少的情况 (比如输出5个就填5个)
                size_t fill_count = std::min(future_states.size(), static_cast<size_t>(array_capacity));

                // 填充 Targets (假设 msg 里的 Target 类型和你的 TargetState 有对应关系)
                for (size_t i = 0; i < fill_count; ++i) {
                    target_array_msg->targets[i] = state_to_ros_msg(
                            future_states[i], 
                            now_ns, 
                            i + 1, 
                            sample_interval_ns_
                        );
                }

                target_array_pub_->publish(std::move(target_array_msg));
            }
            
            last_predict_time_ns_ = now_ns;
        }
    }
}

void TargetStatePredictor::run() {
    // 1. 获取初始时间
    uint64_t start_time = data_manager_->now_ns();
    
    // 初始化时间控制变量 (或者由第一次 tick 内部处理)
    next_sample_time_ns_ = 0; 

    while (is_running_) {
        // Online 模式：时间来源是 DataManager 的实时时间 (通常是系统时间)
        uint64_t now_ns = data_manager_->now_ns();

        // 调用核心逻辑
        tick(now_ns);

        // --- 智能休眠逻辑 ---
        uint64_t after_work_ns = data_manager_->now_ns();
        int64_t wait_ns = static_cast<int64_t>(next_sample_time_ns_) - static_cast<int64_t>(after_work_ns);

        if (wait_ns > 2000000) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(wait_ns - 1000000));
        } else if (wait_ns > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        } else {
            std::this_thread::yield();
        }
    }
}

void TargetStatePredictor::load_model(const std::string& model_path) {

    Ort::SessionOptions session_options;
    OrtCUDAProviderOptions cuda_options;

    cuda_options.device_id = 0;
    cuda_options.arena_extend_strategy = 0;
    cuda_options.gpu_mem_limit = static_cast<size_t>(6) * 1024 * 1024 * 1024; // 4GB
    cuda_options.do_copy_in_default_stream = 1;
    session_options.AppendExecutionProvider_CUDA(cuda_options);

    try {
        predictor_model_.session = Ort::Session(ort_env_, model_path.c_str(), session_options);

        Ort::AllocatorWithDefaultOptions allocator;

        size_t num_input_nodes = predictor_model_.session.GetInputCount();
        for (size_t i = 0; i < num_input_nodes; i++) {
            auto input_name = predictor_model_.session.GetInputNameAllocated(i, allocator);
            std::string name_str(input_name.get());
            predictor_model_.input_names_str.push_back(name_str);
            predictor_model_.input_names.push_back(predictor_model_.input_names_str.back().c_str());
        }

        // 对 Output 做同样的处理
        size_t num_output_nodes = predictor_model_.session.GetOutputCount();
        for (size_t i = 0; i < num_output_nodes; i++) {
            auto output_name = predictor_model_.session.GetOutputNameAllocated(i, allocator);
            std::string name_str(output_name.get());
            predictor_model_.output_names_str.push_back(name_str);
            predictor_model_.output_names.push_back(predictor_model_.output_names_str.back().c_str());
        }

        // 3. 获取维度信息 [Batch, Seq_Len, Vars]
        auto input_dims = predictor_model_.session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        auto output_dims = predictor_model_.session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();

        // 更新参数
        predictor_model_.batch_size = input_dims[0] > 0 ? input_dims[0] : 1;
        predictor_model_.seq_len    = input_dims[1]; // 通常是 450
        predictor_model_.n_vars     = input_dims[2]; // 变量数
        predictor_model_.pred_len   = output_dims[1]; // 通常是 75

        // 同步更新类的控制变量
        max_history_size_ = predictor_model_.seq_len;
        prediction_count_ = predictor_model_.pred_len;

        std::cout << "[TargetStatePredictor] Model Loaded: " << model_path << "\n"
                  << "  Input: [" << predictor_model_.batch_size << ", " << predictor_model_.seq_len << ", " << predictor_model_.n_vars << "]\n"
                  << "  Output: [" << predictor_model_.batch_size << ", " << predictor_model_.pred_len << ", " << predictor_model_.n_vars << "]" 
                  << std::endl;

    } catch (const Ort::Exception& e) {
        std::cerr << "[TargetStatePredictor] Failed to load model: " << e.what() << std::endl;
        // 根据需求决定是否 throw 或者设置错误标志
    }
}

// 核心预测函数
std::vector<TargetState> TargetStatePredictor::predict_future(const std::deque<TargetState>& history) {
    std::vector<TargetState> future_states;

    if (!predictor_model_.session) {
        std::cerr << "Model session not initialized!" << std::endl;
        return future_states;
    }
    
    if (history.size() != static_cast<size_t>(predictor_model_.seq_len)) {
        std::cerr << "History buffer size mismatch! Expected " << predictor_model_.seq_len 
                  << ", got " << history.size() << std::endl;
        return future_states;
    }

    // 1. 准备输入数据 (Flatten)
    // 大小 = 1 * seq_len * n_vars
    size_t input_tensor_size = 1 * predictor_model_.seq_len * predictor_model_.n_vars;
    std::vector<float> input_data;
    input_data.reserve(input_tensor_size);

    for (const auto& state : history) {
        state_to_features(state, input_data);
    }

    // 2. 创建 Tensor
    std::vector<int64_t> input_shape = {1, predictor_model_.seq_len, predictor_model_.n_vars};
    
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_data.data(), input_data.size(), 
        input_shape.data(), input_shape.size());

    // 3. 运行推理
    try {
        auto output_tensors = predictor_model_.session.Run(
            Ort::RunOptions{nullptr}, 
            predictor_model_.input_names.data(), &input_tensor, 1, 
            predictor_model_.output_names.data(), 1
        );

        if (output_tensors.empty()) return future_states;

        // 4. 处理输出
        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        
        // 获取最后一个历史的时间戳作为基准
        uint64_t base_time = history.back().timestamp;
        
        for (int i = 0; i < predictor_model_.pred_len; ++i) {
            // 指针偏移：第 i 个时间步的起始位置
            float* current_step_data = output_data + (i * predictor_model_.n_vars);
            
            // 将 float 数组转回 TargetState
            TargetState pred_state = features_to_state(current_step_data, base_time, i + 1);
            future_states.push_back(pred_state);
        }
        
        // 5. 保存数据到二进制文件 (Log)
        // 放在这里调用，确保每次预测都记录
        save_to_binary(history, future_states);

    } catch (const Ort::Exception& e) {
        std::cerr << "Inference error: " << e.what() << std::endl;
    }

    return future_states;
}

void TargetStatePredictor::state_to_features(const TargetState& state, std::vector<float>& features) {
    features.push_back(static_cast<float>(state.position[0]));
    features.push_back(static_cast<float>(state.position[1]));
    features.push_back(static_cast<float>(state.position[2]));
    features.push_back(static_cast<float>(state.angular_rad[0]));
    features.push_back(static_cast<float>(state.angular_rad[1]));
    features.push_back(static_cast<float>(state.angular_rad[2]));
}
TargetState TargetStatePredictor::features_to_state(const float* f, uint64_t base_time, int step_index) {
    TargetState state;
    // 计算预测帧的时间戳 (15Hz -> 66.6ms)
    state.timestamp = base_time + (step_index * sample_interval_ns_);

    state.position[0] = f[0];
    state.position[1] = f[1];
    state.position[2] = f[2];
    state.angular_rad[0] = f[3];
    state.angular_rad[1] = f[4];
    state.angular_rad[2] = f[5];
    
    return state;
}
prometheus_msgs::msg::Target TargetStatePredictor::state_to_ros_msg(
    const TargetState& state, 
    uint64_t base_time_ns, 
    int step_idx, 
    uint64_t dt_ns) 
{
    prometheus_msgs::msg::Target msg;

    // 2. 计算该预测点对应的时间戳
    // 每一个预测点的时间 = 当前时间 + (第几步 * 步长)
    uint64_t target_time = base_time_ns + (step_idx * dt_ns);
    msg.timestamp.stamp = rclcpp::Time(static_cast<int64_t>(target_time));

    // 3. 位置 (Position)
    msg.px = state.position[0];
    msg.py = state.position[1];
    msg.pz = state.position[2];

    msg.attitude[0] = state.angular_rad[0]; // Roll
    msg.attitude[1] = state.angular_rad[1]; // Pitch
    msg.attitude[2] = state.angular_rad[2]; // Yaw

    // 四元数填充
    msg.attitude_q.x = state.orientation.x();
    msg.attitude_q.y = state.orientation.y();
    msg.attitude_q.z = state.orientation.z();
    msg.attitude_q.w = state.orientation.w();

    return msg;
}

void TargetStatePredictor::save_to_binary(const std::deque<TargetState>& history, const std::vector<TargetState>& future) {
    // 使用 append 模式打开
    std::string filename = data_manager_->get_config()->predictor_file_;
    std::ofstream ofs(filename, std::ios::binary | std::ios::app);
    
    if (!ofs.is_open()) return;

    // 为了方便 Python 解析，建议保存 float 数组而不是 struct
    // 格式: [MagicNum] [Timestamp] [Input(450*9 floats)] [Output(75*9 floats)]
    
    uint64_t magic = 0xAABBCCDD; // 分隔符 (可选)
    uint64_t ts = history.back().timestamp; // 当前预测时刻
    
    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&ts), sizeof(ts));

    // 1. 写入 History (Input)
    std::vector<float> input_flat;
    for (const auto& s : history) {
        state_to_features(s, input_flat);
    }
    ofs.write(reinterpret_cast<const char*>(input_flat.data()), input_flat.size() * sizeof(float));

    // 2. 写入 Future (Output)
    std::vector<float> output_flat;
    for (const auto& s : future) {
        state_to_features(s, output_flat);
    }
    ofs.write(reinterpret_cast<const char*>(output_flat.data()), output_flat.size() * sizeof(float));
    
    ofs.close();
}