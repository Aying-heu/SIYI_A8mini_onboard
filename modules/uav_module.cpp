#include "uav_module.hpp"
#include "data_manager.hpp" // 在 .cpp 里才真正包含 DataManager 的细节
#include "data_types.hpp"

UavModule::UavModule(std::shared_ptr<DataManager> data_manager) 
    : Node("uav_subscriber_node"), data_manager_(data_manager) 
{
    // 创建订阅
    subscription_ = this->create_subscription<prometheus_msgs::msg::UAVState>(
        "uav1/prometheus/state", // 话题名称
        10,                      // 队列深度
        std::bind(&UavModule::uav_state_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "UAV State Subscriber Module initialized.");
}

void UavModule::uav_state_callback(const prometheus_msgs::msg::UAVState::SharedPtr msg) {
    // 1. 获取时间戳（使用 DataManager 里的统一时钟）
    uint64_t ts = data_manager_->now_ns();

    // 2. 将 ROS 消息转换为我们定义的 UAV_Row 结构体（简化数据，方便后续计算）
    UAV_Row row;
    row.nano_timestamp = ts;
    row.position_x = msg->position[0];
    row.position_y = msg->position[1];
    row.position_z = msg->position[2];
    row.velocity_x = msg->velocity[0];
    row.velocity_y = msg->velocity[1];
    row.velocity_z = msg->velocity[2];
    row.attitude_qx = msg->attitude_q.x;
    row.attitude_qy = msg->attitude_q.y;
    row.attitude_qz = msg->attitude_q.z;
    row.attitude_qw = msg->attitude_q.w;
    row.attitude_roll = msg->attitude[0]; // 假设 Prometheus 消息里存的是弧度
    row.attitude_pitch = msg->attitude[1];
    row.attitude_yaw = msg->attitude[2];
    row.thrust_1 = msg->motor_outputs[0];
    row.thrust_2 = msg->motor_outputs[1];
    row.thrust_3 = msg->motor_outputs[2];
    row.thrust_4 = msg->motor_outputs[3];

    // 3. 将数据推入 DataManager 的历史 map 中
    data_manager_->pushUavState(row);
}