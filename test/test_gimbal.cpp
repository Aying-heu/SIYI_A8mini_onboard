#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

// --- SIYI A8 mini 协议相关 ---
#define SERVER_IP "192.168.1.25" // 云台默认IP
#define SERVER_PORT 37260          // 云台默认端口


const uint16_t crc16_tab[256]=
    {0x0,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0xa50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0xc60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0xe70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0xa1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x2b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x8e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0xaf1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0xcc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0xed1,0x1ef0
    };

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

// 发送角度控制包 (CmdID: 0x0E)
void send_gimbal_angle(int sockfd, struct sockaddr_in &addr, float yaw, float pitch) {
    int16_t y_val = static_cast<int16_t>(yaw * 10.0f);
    int16_t p_val = static_cast<int16_t>(pitch * 10.0f);

    uint8_t data_without_crc[12] = {
        0x55, 0x66, 0x01, 0x04, 0x00, 0x00, 0x00, 0x0E,
        (uint8_t)(y_val & 0xFF), (uint8_t)((y_val >> 8) & 0xFF),
        (uint8_t)(p_val & 0xFF), (uint8_t)((p_val >> 8) & 0xFF)
    };

    uint16_t crc = CRC16_cal(data_without_crc, 12, 0);
    unsigned char send_buf[14];
    memcpy(send_buf, data_without_crc, 12);
    send_buf[12] = crc & 0xFF;
    send_buf[13] = (crc >> 8) & 0xFF;

    sendto(sockfd, send_buf, sizeof(send_buf), 0, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
}

int main() {
    // 1. 初始化 UDP Socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in gimbal_addr;
    memset(&gimbal_addr, 0, sizeof(gimbal_addr));
    gimbal_addr.sin_family = AF_INET;
    gimbal_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    gimbal_addr.sin_port = htons(SERVER_PORT);
    // gimbal_addr.addr_len = sizeof(struct sockaddr_in);

    // 2. GStreamer 管道 (Xavier 硬件加速)
    std::string pipeline = 
        "v4l2src device=/dev/video0 ! "
        "image/jpeg, width=2560, height=1440, framerate=30/1 ! "
        "nvv4l2decoder mjpeg=1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "无法打开相机!" << std::endl;
        return -1;
    }

    cv::namedWindow("Test", cv::WINDOW_NORMAL);
    cv::resizeWindow("Test", 1280, 720);

    bool is_sin_cos_active = false;
    auto start_time = std::chrono::steady_clock::now();
    
    std::cout << "\n--- 云台测试模式开启 ---" << std::endl;
    std::cout << "[Space]: 发送单次转向报文 (Yaw:30, Pitch:-20)" << std::endl;
    std::cout << "[X]:     开启/关闭 Sin-Cos 简谐运动" << std::endl;
    std::cout << "[Q]:     退出程序" << std::endl;

    cv::Mat frame;

    float sign=-1.0;

    float yaw_plus = -2.0f;;
    float pitch_plus = 2.0f;
    float now_yaw = 20.0f;
    float now_pitch=-30.0f;

    while (true) {
        if (!cap.read(frame)) break;

        // 3. 处理按键
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') break;

        // [Space] 单次控制测试
        if (key == ' ') {
            sign*=-1.0;
            std::cout << ">> [SPACE] 发送控制指令!" << std::endl;
            send_gimbal_angle(sockfd, gimbal_addr, 20.0f * sign, -30.0f * sign);
        }
        if (key == 'b' || key == 'B') {
            std::cout << ">> [B] 发送控制指令!" << std::endl;
            if(now_yaw  >= 19.0f )yaw_plus=-2.0f;
            if(now_pitch  >= 29.0f )pitch_plus=-2.0f;
            if(now_yaw  <= -19.0f )yaw_plus=2.0f;
            if(now_pitch  <= -19.0f )pitch_plus=2.0f;

            now_yaw+=yaw_plus;
            now_pitch+=pitch_plus;

            send_gimbal_angle(sockfd, gimbal_addr, now_yaw, now_pitch);
        }

        // [X] 简谐运动开关
        if (key == 'x' || key == 'X') {
            is_sin_cos_active = !is_sin_cos_active;
            std::cout << ">> [X] 简谐运动状态: " << (is_sin_cos_active ? "开启" : "停止") << std::endl;
        }

        // 4. 执行自动运动逻辑
        if (is_sin_cos_active) {
            auto now = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() / 1000.0f;
            
            // 计算 Sin Cos 角度 (范围 +/- 20度)
            float yaw_cmd = 20.0f * std::sin(elapsed * 2.0f);   // 周期约 3.14s
            float pitch_cmd = 30.0f * std::cos(elapsed * 2.0f);
            
            send_gimbal_angle(sockfd, gimbal_addr, yaw_cmd, pitch_cmd);
            
            // 在图像上标记
            cv::putText(frame, "AUTO MOVING: ON", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("Test", frame);
    }

    // 停止运动并归中
    send_gimbal_angle(sockfd, gimbal_addr, 0, 0);
    cap.release();
    close(sockfd);
    return 0;
}