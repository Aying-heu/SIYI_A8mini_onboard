#include "gimbal_module.hpp"
#include <prometheus_msgs/msg/target.hpp>
#include <cmath>

/***********************************************************
 * CRC16 校验代码
 * CRC16 Coding & Decoding G(X) = X^16+X^12+X^5+1
***********************************************************/
uint16_t CRC16_cal(uint8_t *ptr, uint32_t len, uint16_t crc_init)
{
    uint16_t crc, oldcrc16;
    uint8_t temp;
    crc = crc_init;
    while (len--!=0)
    {
        temp=(crc>>8)&0xff;
        oldcrc16=crc16_tab[*ptr^temp];
        crc=(crc<<8)^oldcrc16;
        ptr++;
    }
    //crc=~crc; //??
    return(crc);
}
uint8_t crc_check_16bites(uint8_t* pbuf, uint32_t len,uint32_t* p_result)
{
    uint16_t crc_result = 0;
    crc_result= CRC16_cal(pbuf,len, 0);
    *p_result = crc_result;
    return 2;
}


GimbalModule::GimbalModule(std::shared_ptr<DataManager> data_manager):data_manager_(data_manager){
    initializeGimbalCommunicate();
    _shutdown_ = false;
}

GimbalModule::~GimbalModule(){
    _shutdown_ = true;
    running_ = false;
    if (gimbal_poll_thread_.joinable()) gimbal_poll_thread_.join();
}
bool GimbalModule::start(){
    if (running_) {
        return false;
    }
    running_ = true;
    // 启动云台状态轮询线程（20Hz）
    gimbal_poll_thread_ = std::thread(&GimbalModule::gimbalPollLoop, this);
    return true;
}

void GimbalModule::initializeGimbalCommunicate(){
    if ((Communicate_pipe.sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket");
        exit(1);
    }
    memset(&Communicate_pipe.send_addr, 0, sizeof(Communicate_pipe.send_addr));
    Communicate_pipe.send_addr.sin_family = AF_INET;
    Communicate_pipe.send_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    Communicate_pipe.send_addr.sin_port = htons(SERVER_PORT);
    Communicate_pipe.addr_len = sizeof(struct sockaddr_in);
    // 设置接收超时，避免 recvfrom 长时间阻塞
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1000; // 40 ms  =  40000
    setsockopt(Communicate_pipe.sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    // unsigned char send_buf[11] = {
    //     0x55, 0x66,
    //     0x01,
    //     0x01, 0x00,
    //     0x00, 0x00,
    //     0x0c,
    //     0x03, 0x57,0xfe
    // };

    unsigned char send_buf[11] = {
        0x55, 0x66,
        0x01,
        0x01, 0x00,
        0x00, 0x00,
        0x0c,
        0x04, 0xb0,0x8e
    };
    sendto(Communicate_pipe.sockfd, send_buf, sizeof(send_buf), 0, 
           (struct sockaddr*)&Communicate_pipe.send_addr, Communicate_pipe.addr_len);
}
void GimbalModule::enableGimbalStream(){
    // 发送0x25 设置推流模式
    unsigned char data_without_crc[10] = {
        0x55, 0x66,
        0x01,
        0x02, 0x00,
        0x00, 0x00,
        0x25,
        0x01, 0x05,
    };
    uint16_t crc = CRC16_cal(data_without_crc, 10, 0);
    unsigned char send_buf[12];
    memcpy(send_buf, data_without_crc, 10);
    send_buf[10] = crc & 0xFF;
    send_buf[11] = (crc >> 8) & 0xFF;

    sendto(Communicate_pipe.sockfd, send_buf, sizeof(send_buf), 0, 
           (struct sockaddr*)&Communicate_pipe.send_addr, Communicate_pipe.addr_len);
}

// void GimbalModule::gimbalPollLoop(){
//     const std::chrono::milliseconds set_Stream_ms(10000); // 每10秒发送一次，确保心跳
//     enableGimbalStream();
//     auto t0 = std::chrono::steady_clock::now();
//     while (running_ && !_shutdown_) {
//         get_gimbalstate();
//         if(idx==5){
//             control_gimbalstate();
//             idx=0;
//         }
//         auto t1 = std::chrono::steady_clock::now();
//         auto used = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
//         if (used < period_ms) std::this_thread::sleep_for(1);
//     }
// }
void GimbalModule::gimbalPollLoop(){
    auto last_stream_time = std::chrono::steady_clock::now();
    auto last_zoom_time = std::chrono::steady_clock::now();
    int idx = 0;

    // 初始开启推流
    enableGimbalStream();

    while (running_ && !_shutdown_) {
        auto t_start = std::chrono::steady_clock::now();

        // 1. 检查心跳（10秒一次）
        if (t_start - last_stream_time > std::chrono::seconds(10)) {
            enableGimbalStream();
            last_stream_time = t_start;
        }

        // 2. 定时请求变倍（1秒一次，减少网络负担）
        if (t_start - last_zoom_time > std::chrono::seconds(1)) {
            unsigned char get_zoom_req[10] = {0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x18,0x7C,0x47};
            sendto(Communicate_pipe.sockfd, get_zoom_req, 10, 0, (struct sockaddr*)&Communicate_pipe.send_addr, Communicate_pipe.addr_len);
            last_zoom_time = t_start;
        }

        // 3. 读取推流数据
        get_gimbalstate();

        // // 4. 控制逻辑 (假设每 5 次读取控制一次，即 4Hz)
        // idx++;
        // if(idx >= 5){
        //     control_gimbalstate();
        //     idx = 0;
        // }

        // 5. 保持循环频率 (20Hz -> 50ms)
        auto t_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
        if (elapsed < std::chrono::milliseconds(50)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50) - elapsed);
        }
    }
}

// void GimbalModule::get_gimbalstate() {
//     // 1. 发送请求（保持不变）
//     unsigned char get_zoom_req[10] = {0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x18,0x7C,0x47};
//     unsigned char get_rpy_req[10] = {0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x0d,0xe8,0x05};

//     sendto(Communicate_pipe.sockfd, get_zoom_req, 10, 0, (struct sockaddr*)&Communicate_pipe.send_addr, Communicate_pipe.addr_len);
//     sendto(Communicate_pipe.sockfd, get_rpy_req, 10, 0, (struct sockaddr*)&Communicate_pipe.send_addr, Communicate_pipe.addr_len);

//     // 临时变量，用于判断本轮是否更新了数据
//     bool got_zoom = false;
//     bool got_rpy = false;
//     // 2. 循环接收所有等待中的包
//     unsigned char recv_buf[RECV_BUF_SIZE];
    
//     // 注意：这里的循环能保证你把当前内核缓冲区里所有的云台回包都处理掉
//     uint64_t got_time = data_manager_->now_ns();
//     while (true) {
//         int len = recvfrom(Communicate_pipe.sockfd, recv_buf, RECV_BUF_SIZE, 0,
//                            (struct sockaddr *)&Communicate_pipe.recv_addr, &Communicate_pipe.addr_len);
        
//         if (len <= 0) break; // 超时或无数据，退出循环

//         // 校验包头
//         if (len < 10 || recv_buf[0] != 0x55 || recv_buf[1] != 0x66) continue;

//         uint8_t cmd_id = recv_buf[7];
//         uint16_t data_len = static_cast<uint8_t>(recv_buf[3]) | (static_cast<uint8_t>(recv_buf[4]) << 8);

//         if (cmd_id == 0x18 && data_len >= 2) {
//             // 解析 Zoom
//             uint8_t zoom_int = recv_buf[8];
//             uint8_t zoom_dec = recv_buf[9];
//             gimbal_state_.zoom =static_cast<float>(zoom_int) + static_cast<float>(zoom_dec) / 10.0f;
//             got_zoom = true;
//         }
//         else if (cmd_id == 0x0d && data_len >= 12) {
//             // 解析 RPY
//             int16_t raw_yaw_u = static_cast<int16_t>(recv_buf[8] | (recv_buf[9] << 8));
//             int16_t raw_pitch_u = static_cast<int16_t>(recv_buf[10] | (recv_buf[11] << 8));
//             int16_t raw_roll_u = static_cast<int16_t>(recv_buf[12] | (recv_buf[13] << 8));
//             int16_t raw_yaw_v_u = static_cast<int16_t>(recv_buf[14] | (recv_buf[15] << 8));
//             int16_t raw_pitch_v_u = static_cast<int16_t>(recv_buf[16] | (recv_buf[17] << 8));
//             int16_t raw_roll_v_u = static_cast<int16_t>(recv_buf[18] | (recv_buf[19] << 8));
            
//             // 坐标系转换逻辑
//             int16_t raw_yaw = static_cast<int16_t>(raw_yaw_u);
//             int16_t raw_pitch = static_cast<int16_t>(raw_pitch_u);
//             int16_t raw_roll = static_cast<int16_t>(raw_roll_u);
//             int16_t raw_yaw_velocity = static_cast<int16_t>(raw_yaw_v_u);
//             int16_t raw_pitch_velocity = static_cast<int16_t>(raw_pitch_v_u);
//             int16_t raw_roll_velocity = static_cast<int16_t>(raw_roll_v_u);

//             // 根据协议，角度/角速度以 0.1 单位发送（即发送时乘以10）
//             float gimbal_current_yaw = static_cast<float>(raw_yaw) / 10.0f;
//             float gimbal_current_pitch = static_cast<float>(raw_pitch) / 10.0f;
//             float gimbal_current_roll = static_cast<float>(raw_roll) / 10.0f;
//             float gimbal_current_yaw_velocity = static_cast<float>(raw_yaw_velocity) / 10.0f;
//             float gimbal_current_pitch_velocity = static_cast<float>(raw_pitch_velocity) / 10.0f;
//             float gimbal_current_roll_velocity = static_cast<float>(raw_roll_velocity) / 10.0f;

//             gimbal_current_yaw = - gimbal_current_yaw;
//             if(gimbal_current_pitch>=0) gimbal_current_pitch-=180;
//             else gimbal_current_pitch+=180;

//             gimbal_current_yaw_velocity = - gimbal_current_yaw_velocity;
//             gimbal_current_pitch_velocity = -gimbal_current_pitch_velocity;

//             gimbal_state_.roll = gimbal_current_roll;
//             gimbal_state_.pitch = gimbal_current_pitch;
//             gimbal_state_.yaw = gimbal_current_yaw;
//             gimbal_state_.roll_velocity = gimbal_current_roll_velocity;
//             gimbal_state_.pitch_velocity = gimbal_current_pitch_velocity;
//             gimbal_state_.yaw_velocity = gimbal_current_yaw_velocity;
//             got_rpy = true;
//         }
//         if (got_zoom && got_rpy) break; 
//     }
//     // 3. 只有当本轮拿到了关键的姿态数据时，才推送到 data_manager
//     if (got_rpy) {
//         gimbal_state_.nano_timestamp = got_time;
//         data_manager_->pushGimbalState(gimbal_state_);
//     }
// }

void GimbalModule::get_gimbalstate() {
    bool got_rpy = false;
    bool got_zoom = false;
    unsigned char recv_buf[RECV_BUF_SIZE];
    uint64_t got_time = data_manager_->now_ns();

    // 此时 while 会排空所有推流积压的包，只取最新的
    while (true) {
        int len = recvfrom(Communicate_pipe.sockfd, recv_buf, RECV_BUF_SIZE, 0,
                           (struct sockaddr *)&Communicate_pipe.recv_addr, &Communicate_pipe.addr_len);
        
        if (len <= 0) break; 
        if (len < 10 || recv_buf[0] != 0x55 || recv_buf[1] != 0x66) continue;

        uint8_t cmd_id = recv_buf[7];
        if (cmd_id == 0x18) { // 变倍回复
            gimbal_state_.zoom = static_cast<float>(recv_buf[8]) + static_cast<float>(recv_buf[9]) / 10.0f;
            got_zoom = true;
        }
        else if (cmd_id == 0x0d) { // 姿态回复（推流包）
            int16_t raw_yaw_u = static_cast<int16_t>(recv_buf[8] | (recv_buf[9] << 8));
            int16_t raw_pitch_u = static_cast<int16_t>(recv_buf[10] | (recv_buf[11] << 8));
            int16_t raw_roll_u = static_cast<int16_t>(recv_buf[12] | (recv_buf[13] << 8));
            int16_t raw_yaw_v_u = static_cast<int16_t>(recv_buf[14] | (recv_buf[15] << 8));
            int16_t raw_pitch_v_u = static_cast<int16_t>(recv_buf[16] | (recv_buf[17] << 8));
            int16_t raw_roll_v_u = static_cast<int16_t>(recv_buf[18] | (recv_buf[19] << 8));
            
            // 坐标系转换逻辑
            int16_t raw_yaw = static_cast<int16_t>(raw_yaw_u);
            int16_t raw_pitch = static_cast<int16_t>(raw_pitch_u);
            int16_t raw_roll = static_cast<int16_t>(raw_roll_u);
            int16_t raw_yaw_velocity = static_cast<int16_t>(raw_yaw_v_u);
            int16_t raw_pitch_velocity = static_cast<int16_t>(raw_pitch_v_u);
            int16_t raw_roll_velocity = static_cast<int16_t>(raw_roll_v_u);

            // 根据协议，角度/角速度以 0.1 单位发送（即发送时乘以10）
            float gimbal_current_yaw = static_cast<float>(raw_yaw) / 10.0f;
            float gimbal_current_pitch = static_cast<float>(raw_pitch) / 10.0f;
            float gimbal_current_roll = static_cast<float>(raw_roll) / 10.0f;
            float gimbal_current_yaw_velocity = static_cast<float>(raw_yaw_velocity) / 10.0f;
            float gimbal_current_pitch_velocity = static_cast<float>(raw_pitch_velocity) / 10.0f;
            float gimbal_current_roll_velocity = static_cast<float>(raw_roll_velocity) / 10.0f;

            gimbal_current_yaw = - gimbal_current_yaw;
            if(gimbal_current_pitch>=0) gimbal_current_pitch-=180;
            else gimbal_current_pitch+=180;

            gimbal_current_yaw_velocity = - gimbal_current_yaw_velocity;
            gimbal_current_pitch_velocity = -gimbal_current_pitch_velocity;

            gimbal_state_.roll = gimbal_current_roll;
            gimbal_state_.pitch = gimbal_current_pitch;
            gimbal_state_.yaw = gimbal_current_yaw;
            gimbal_state_.roll_velocity = gimbal_current_roll_velocity;
            gimbal_state_.pitch_velocity = gimbal_current_pitch_velocity;
            gimbal_state_.yaw_velocity = gimbal_current_yaw_velocity;
            got_rpy = true;
        }
        
        // 如果本轮同时拿到了姿态和变倍，可以直接走。
        // 如果没发变倍请求，got_zoom 为 false，会继续读直到缓冲区排空。
        if (got_rpy && got_zoom) break; 
    }

    if (got_rpy) {
        gimbal_state_.nano_timestamp = got_time;
        data_manager_->pushGimbalState(gimbal_state_);
    }
}


void GimbalModule::control_gimbalstate() {
    UAV_Row uav_now;
    if (!data_manager_->getLatestUavState(uav_now)) return;
    prometheus_msgs::msg::Target target_msg;
    if (!data_manager_->getLatestTargetMsg(target_msg)) return;


    const double rad2deg = 180.0 / M_PI;
    const double deg2rad = M_PI / 180.0;
    
    double target_x = target_msg.px;
    double target_y = target_msg.py;
    double target_z = target_msg.pz;

    double now_uav_cam_x = uav_now.position_x;
    double now_uav_cam_y = uav_now.position_y;
    double now_uav_cam_z = uav_now.position_z;

    // --- 4. 计算相对矢量 (融合当前与预测，平滑跟踪) ---
    double dx = target_x - (now_uav_cam_x);
    double dy = target_y - (now_uav_cam_y);
    double dz = target_z - (now_uav_cam_z);

    // --- 5. 解算云台目标角度 ---
    // gimbal_target_yaw 是相对于地理北向的角度，如果是Follow模式需减去无人机yaw
    // 注意：根据你的云台协议，确定是发送绝对角度还是相对角度
    double gimbal_target_yaw_global = atan2(dy, dx); 
    double gimbal_target_yaw = gimbal_target_yaw_global - uav_now.attitude_yaw; // 转化为相对于机头的角度

    double horizontal_dist = sqrt(dx * dx + dy * dy);
    double gimbal_target_pitch = atan2(dz, horizontal_dist);

    // 转换为角度
    // 注意：此处正负号需根据云台硬件定义调整（通常：抬头为正，右转为正）
    double g_yaw_deg = -gimbal_target_yaw * rad2deg;
    double g_pitch_deg = -gimbal_target_pitch * rad2deg;

    // --- 6. 角度归一化 (修正：应使用360度) ---
    auto normalize_angle = [](double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    };
    g_yaw_deg = normalize_angle(g_yaw_deg);
    g_pitch_deg = normalize_angle(g_pitch_deg);

    // --- 7. 构建协议并发送 ---
    int16_t yaw_value = (int16_t)(g_yaw_deg * 10);
    int16_t pitch_value = (int16_t)(g_pitch_deg * 10);

    unsigned char data_without_crc[12] = {
        0x55, 0x66,
        0x01,
        0x04, 0x00,
        0x00, 0x00,
        0x0E,
        (uint8_t)(yaw_value & 0xFF), (uint8_t)((yaw_value >> 8) & 0xFF),
        (uint8_t)(pitch_value & 0xFF), (uint8_t)((pitch_value >> 8) & 0xFF)
    };

    uint16_t crc = CRC16_cal(data_without_crc, 12, 0);
    unsigned char send_buf[14];
    memcpy(send_buf, data_without_crc, 12);
    send_buf[12] = crc & 0xFF;
    send_buf[13] = (crc >> 8) & 0xFF;

    sendto(Communicate_pipe.sockfd, send_buf, sizeof(send_buf), 0, 
           (struct sockaddr*)&Communicate_pipe.send_addr, Communicate_pipe.addr_len);
}