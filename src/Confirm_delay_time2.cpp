#include "data_types.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <chrono>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <prometheus_msgs/msg/target.hpp>

struct DtScore {
    uint64_t dt_ns;
    double score;
};

// 计算标准差的函数
double calculateStdDev(const std::vector<double>& values) {
    if (values.empty()) return 999999.0;
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    double mean = sum / values.size();
    double sq_sum = std::inner_product(values.begin(), values.end(), values.begin(), 0.0);
    return std::sqrt(sq_sum / values.size() - mean * mean);
}
double calculateErr(const std::vector<double>& values) {
    // 空数据或只有1个数据点，无相邻差值，返回0
    if (values.empty() || values.size() == 1) {
        return 0.0;
    }

    double abs_diff_sum = 0.0;
    // 遍历相邻数据点，计算差值的绝对值并累加
    for (size_t i = 1; i < values.size(); ++i) {
        // 相邻两点的差值（当前点 - 前一个点）
        double diff = values[i] - values[i-1];
        // 取绝对值后累加
        if(std::fabs(diff)<0.2)continue;
        abs_diff_sum += std::fabs(diff);
    }

    return abs_diff_sum;
}
// 1. 读取无人机状态（严格对应 19 列）
std::map<uint64_t,UAV_Row> loadOfflineUavStates(const std::string& uav_state_file) {
    std::map<uint64_t,UAV_Row> states;
    std::ifstream file(uav_state_file);
    std::string line;
    
    std::getline(file, line); // 跳过表头

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> columns;
        
        while (std::getline(ss, token, ',')) {
            columns.push_back(token);
        }

        if (columns.size() < 19) continue; // 确保列数完整

        try {
            UAV_Row row;
            // 时间戳：必须 stoull
            row.nano_timestamp = std::stoull(columns[0]); 
            
            // 位置
            row.position_x = std::stof(columns[2]);
            row.position_y = std::stof(columns[3]);
            row.position_z = std::stof(columns[4]);

            // 速度
            row.velocity_x = std::stof(columns[5]);
            row.velocity_y = std::stof(columns[6]);
            row.velocity_z = std::stof(columns[7]);

            // 四元数 (qx, qy, qz, qw)
            row.attitude_qx = std::stof(columns[8]);
            row.attitude_qy = std::stof(columns[9]); // 之前漏掉了这一列
            row.attitude_qz = std::stof(columns[10]);
            row.attitude_qw = std::stof(columns[11]);

            // 欧拉角 (roll, pitch, yaw)
            row.attitude_roll  = std::stof(columns[12]);
            row.attitude_pitch = std::stof(columns[13]);
            row.attitude_yaw   = std::stof(columns[14]);

            // 推力 (1, 2, 3, 4)
            row.thrust_1 = std::stof(columns[15]);
            row.thrust_2 = std::stof(columns[16]);
            row.thrust_3 = std::stof(columns[17]);
            row.thrust_4 = std::stof(columns[18]);

            states[row.nano_timestamp]=row;
        } catch (...) { continue; }
    }
    return states;
}

// 2. 读取云台状态（严格对应 9 列）
std::map<uint64_t,Gimbal_Row> loadOfflineGimbalStates(const std::string& gimbal_state_file) {
    std::map<uint64_t,Gimbal_Row> states;
    std::ifstream file(gimbal_state_file);
    std::string line;

    std::getline(file, line); // 跳过表头

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> columns;

        while (std::getline(ss, token, ',')) {
            columns.push_back(token);
        }

        if (columns.size() < 9) continue;

        try {
            Gimbal_Row row;
            row.nano_timestamp = std::stoull(columns[0]);
            row.zoom           = std::stof(columns[2]);
            row.roll           = std::stof(columns[3]);
            row.pitch          = std::stof(columns[4]);
            row.yaw            = std::stof(columns[5]);
            row.roll_velocity  = std::stof(columns[6]);
            row.pitch_velocity = std::stof(columns[7]);
            row.yaw_velocity   = std::stof(columns[8]);
            states[row.nano_timestamp]=row;
        } catch (...) { continue; }
    }
    return states;
}

std::vector<prometheus_msgs::msg::Target> loadOfflineTargetStates(const std::string& target_state_file) {
    std::vector<prometheus_msgs::msg::Target> states;
    std::ifstream file(target_state_file);
    std::string line;

    std::getline(file, line); // 跳过表头

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> columns;

        while (std::getline(ss, token, ',')) {
            columns.push_back(token);
        }

        if (columns.size() < 9) continue;

        try {
            prometheus_msgs::msg::Target row;
            
            uint64_t ts_ns = std::stoull(columns[0]);
            row.timestamp.stamp=rclcpp::Time(ts_ns);
            row.px           = std::stof(columns[2]);
            row.py           = std::stof(columns[3]);
            row.pz          = std::stof(columns[4]);
            row.attitude[0]            = std::stof(columns[5]);
            row.attitude[1]  = std::stof(columns[6]);
            row.attitude[2] = std::stof(columns[7]);
            row.attitude_q.x   = std::stof(columns[8]);
            row.attitude_q.y   = std::stof(columns[9]);
            row.attitude_q.z   = std::stof(columns[10]);
            row.attitude_q.w   = std::stof(columns[11]);
            states.push_back(row);
        } catch (...) { continue; }
    }
    return states;
}

bool getUavStateAt(uint64_t ts, UAV_Row& out_state,const std::map<uint64_t,UAV_Row>& uav_states){
    auto it_boundary = uav_states.lower_bound(ts);
    if (it_boundary == uav_states.begin()) {
        std::cerr << "无匹配的无人机状态，时间戳：" << ts << std::endl;
        return false;
    }
    auto it_closest = std::prev(it_boundary);
    out_state = it_closest->second; 
    return true;
}
bool getGimbalStateAt(uint64_t ts, Gimbal_Row& out_state, const std::map<uint64_t,Gimbal_Row>& gimbal_states) {
    auto it_boundary = gimbal_states.lower_bound(ts);
    if (it_boundary == gimbal_states.begin()) {
        std::cerr << "无匹配的云台状态，时间戳：" << ts << std::endl;
        return false;
    }
    auto it_closest = std::prev(it_boundary);
    out_state = it_closest->second; 
    return true;
}

// 从旋转矩阵提取欧拉角（Z-Y-X顺序：yaw-pitch-roll）
void rotationMatrixToEuler(const Eigen::Matrix3d& R, Eigen::Vector3d& euler_rad) {
    double sy = sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
    
    if (sy > 1e-6) {  // 非万向锁
        euler_rad[0] = atan2(R(1,0), R(0,0));   // yaw (Z)
        euler_rad[1] = atan2(-R(2,0), sy);      // pitch (Y)
        euler_rad[2] = atan2(R(2,1), R(2,2));   // roll (X)
    } else {  // 万向锁
        euler_rad[0] = atan2(-R(0,1), R(1,1));  // yaw (Z)
        euler_rad[1] = atan2(-R(2,0), sy);      // pitch (Y)
        euler_rad[2] = 0.0;                     // roll (X)
    }
}
// --------------- 主函数核心逻辑 ---------------
int main(int argc, char** argv) {

    std::string offline_data_root="/home/bsa/A_vision_relate/data_20260114_153832/";

    // 加载离线数据
    std::string uav_state_file = offline_data_root + "/uav_states.csv";
    std::string gimbal_state_file = offline_data_root + "/gimbal_states.csv";
    std::string old_target_state_file = offline_data_root + "/target_states.csv";

    std::map<uint64_t,UAV_Row> uav_states = loadOfflineUavStates(uav_state_file);
    std::map<uint64_t,Gimbal_Row> gimbal_states = loadOfflineGimbalStates(gimbal_state_file);
    std::vector<prometheus_msgs::msg::Target> old_target_states = loadOfflineTargetStates(old_target_state_file);

    std::ofstream target_file_;

    const double rad2deg = 180.0 / M_PI;
    const double deg2rad = M_PI / 180.0;

    const Eigen::Vector3d cam2uav(0.23, 0.0, -0.02);
    Eigen::Matrix3d R_cam2gimbal;
    R_cam2gimbal << 0, -1,  0,
                     0,  0, -1,
                     1,  0,  0;
    std::vector<DtScore> score_results;
    for(uint64_t dt = 10000000;dt<500000000+1;dt+=10000000){
        std::vector<double> z_list;
        target_file_.open(offline_data_root+"/new_target/new_target_states_"+std::to_string(dt)+".csv", std::ios::app);
        target_file_ << "timestamp_ns,dNano_t,px,py,pz,attitude0,attitude1,attitude2,attitude_q_x,attitude_q_y,attitude_q_z,attitude_q_w\n";

        uint64_t oo=0;
        for (prometheus_msgs::msg::Target t : old_target_states){
            uint64_t old_timestamp = static_cast<uint64_t>(t.timestamp.stamp.sec) * 1000000000ULL + t.timestamp.stamp.nanosec;
            uint64_t new_timestamp = old_timestamp + 200000000 - dt;

            UAV_Row old_uav_row, new_uav_row;
            Gimbal_Row old_gimbal_row, new_gimbal_row;                                                                       
            if(!getUavStateAt(old_timestamp,old_uav_row,uav_states))break;
            if(!getUavStateAt(new_timestamp,new_uav_row,uav_states))break;
            if(!getGimbalStateAt(old_timestamp,old_gimbal_row,gimbal_states))break;
            if(!getGimbalStateAt(new_timestamp,new_gimbal_row,gimbal_states))break;

            Eigen::Vector3d old_target_in_world{t.px,t.py,t.pz};
            Eigen::Vector3d old_uav_position{old_uav_row.position_x,old_uav_row.position_y,old_uav_row.position_z};
            Eigen::Vector3d new_uav_position{new_uav_row.position_x,new_uav_row.position_y,new_uav_row.position_z};

            Eigen::Matrix3d old_R_uav2world = Eigen::AngleAxisd(old_uav_row.attitude_roll, Eigen::Vector3d::UnitX()).toRotationMatrix()
                                                *Eigen::AngleAxisd(old_uav_row.attitude_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                                                *Eigen::AngleAxisd(old_uav_row.attitude_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Eigen::Matrix3d new_R_uav2world = Eigen::AngleAxisd(new_uav_row.attitude_roll, Eigen::Vector3d::UnitX()).toRotationMatrix()
                                                *Eigen::AngleAxisd(new_uav_row.attitude_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                                                *Eigen::AngleAxisd(new_uav_row.attitude_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            

                                        
            double old_gimbal_roll_rel = old_gimbal_row.roll * deg2rad - old_uav_row.attitude_roll;
            double old_gimbal_pitch_rel = old_gimbal_row.pitch * deg2rad - old_uav_row.attitude_pitch;
            double old_gimbal_yaw_rel = old_gimbal_row.yaw * deg2rad;

            Eigen::Matrix3d old_R_gimbal = 
                Eigen::AngleAxisd(old_gimbal_roll_rel, Eigen::Vector3d::UnitX()).toRotationMatrix() *
                Eigen::AngleAxisd(old_gimbal_pitch_rel, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                Eigen::AngleAxisd(old_gimbal_yaw_rel, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            double new_gimbal_roll_rel = new_gimbal_row.roll * deg2rad - new_uav_row.attitude_roll;
            double new_gimbal_pitch_rel = new_gimbal_row.pitch * deg2rad - new_uav_row.attitude_pitch;
            double new_gimbal_yaw_rel = new_gimbal_row.yaw * deg2rad;

            Eigen::Matrix3d new_R_gimbal = 
                Eigen::AngleAxisd(new_gimbal_roll_rel, Eigen::Vector3d::UnitX()).toRotationMatrix() *
                Eigen::AngleAxisd(new_gimbal_pitch_rel, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                Eigen::AngleAxisd(new_gimbal_yaw_rel, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            Eigen::Matrix3d old_R_cam2uav = old_R_gimbal * R_cam2gimbal.transpose();
            Eigen::Matrix3d new_R_cam2uav = new_R_gimbal * R_cam2gimbal.transpose();
        
            Eigen::Vector3d old_target2uav = old_R_uav2world.inverse() * (old_target_in_world - old_uav_position);
            
            Eigen::Vector3d old_target2cam = old_R_cam2uav.inverse() * (old_target2uav - cam2uav);

            Eigen::Quaterniond old_target_quat(t.attitude_q.w, t.attitude_q.x, t.attitude_q.y, t.attitude_q.z);
            Eigen::Matrix3d old_R_target2world = old_target_quat.toRotationMatrix();
            Eigen::Matrix3d old_R_target2uav = old_R_uav2world.inverse() * old_R_target2world;
            Eigen::Matrix3d old_R_target2cam = old_R_cam2uav.inverse() * old_R_target2uav;

            Eigen::Vector3d new_target2uav = new_R_cam2uav * old_target2cam + cam2uav;
            Eigen::Vector3d new_target_in_world = new_R_uav2world * new_target2uav + new_uav_position;

            Eigen::Matrix3d new_R_target2uav = new_R_cam2uav * old_R_target2cam;
            Eigen::Matrix3d new_R_target_in_world = new_R_uav2world * new_R_target2uav;

            Eigen::Vector3d new_target_euler_rad;
            rotationMatrixToEuler(new_R_target_in_world, new_target_euler_rad);

            Eigen::Quaterniond new_target_quat(new_R_target_in_world);
            new_target_quat.normalize();
            target_file_ << new_timestamp << "," 
                        << new_timestamp-oo << ","
                        << new_target_in_world.x() << ","
                        << new_target_in_world.y() << ","
                        << new_target_in_world.z() << ","
                        << new_target_euler_rad[0] << ","  // yaw (rad)
                        << new_target_euler_rad[1] << ","  // pitch (rad)
                        << new_target_euler_rad[2] << ","  // roll (rad)
                        << new_target_quat.x() << ","
                        << new_target_quat.y() << ","
                        << new_target_quat.z() << ","
                        << new_target_quat.w() << "\n";
            oo=new_timestamp;

            z_list.push_back(new_target_in_world.z());
        }
        double s = calculateErr(z_list);
        score_results.push_back({dt, s});
        target_file_.close();
        std::cout << "新目标状态已写入：" << offline_data_root+"/new_target/new_target_states_"+std::to_string(dt)+".csv" << std::endl;
    }

    auto best_it = std::min_element(score_results.begin(), score_results.end(), 
               [](const DtScore& a, const DtScore& b){ return a.score < b.score; });

    std::cout << "最佳延迟时间: " << best_it->dt_ns << std::endl;

    return 0;
}





/*

第一次飞行 时间戳手动减 180ms

无人机第一次最低点：
1768628279420340448,19927104,-2.36997,1.74992,1.60038,0.261437,-0.0693735,-1.70874,0.0554809,0.0152482,0.0202321,-0.998138,-0.110421,-0.0326905,-0.0387272,1649,1714,1623,1623
1768628279440318272,19977824,-2.36997,1.74992,1.60038,0.261437,-0.0693735,-1.70874,0.0554809,0.0152482,0.0202321,-0.998138,-0.110421,-0.0326905,-0.0387272,1649,1714,1623,1623
1768628279460429216,20110944,-2.371,1.73592,1.56113,-0.102568,-0.6109,-0.0939185,-0.00681701,0.0420132,0.00354569,-0.999088,0.0139692,-0.0840001,-0.00768489,1649,1714,1623,1623
1768628279480231744,19802528,-2.371,1.73592,1.56113,-0.102568,-0.6109,-0.0939185,-0.00681701,0.0420132,0.00354569,-0.999088,0.0139692,-0.0840001,-0.00768489,1649,1714,1623,1623
1768628279500268768,20037024,-2.37772,1.71458,1.58107,-0.0647459,-0.543072,0.615636,0.00853148,0.030856,-0.000844623,-0.999487,-0.0171398,-0.0617051,0.0022191,1472,1627,1900,1900
1768628279520561056,20292288,-2.37772,1.71458,1.58107,-0.0647459,-0.543072,0.615636,0.0155109,0.0164582,-0.00424318,-0.999735,-0.0311751,-0.032782,0.00899963,1472,1627,1900,1900
1768628279540544352,19983296,-2.38241,1.69608,1.60793,-0.043327,-0.473193,0.692152,0.0333986,0.00185389,-0.0079333,-0.999409,-0.0668372,-0.00317568,0.0159818,1472,1627,1900,1900

无人机第二次最低点：
1768628295920500288,19915072,-2.49674,1.81843,1.60784,0.228546,-0.187655,-2.24707,0.0449261,0.0158472,-0.00383389,-0.998857,-0.0900367,-0.0313189,0.0090875,1675,1663,1629,1629
1768628295940575168,20074880,-2.49674,1.81843,1.60784,0.228546,-0.187655,-2.24707,0.0167786,0.0262653,-0.00391359,-0.999507,-0.0337991,-0.0523973,0.00871678,1675,1663,1629,1629
1768628295960464960,19889792,-2.50089,1.80244,1.58521,-0.161725,-0.361798,0.432317,-0.0121801,0.0510704,-0.00901003,-0.99858,0.0235308,-0.102394,0.0168394,1200,1900,1257,1257
1768628295980528288,20063328,-2.50089,1.80244,1.58521,-0.161725,-0.361798,0.432317,0.01823,0.0503516,-0.0114368,-0.9985,-0.0377556,-0.100303,0.0248023,1200,1900,1257,1257
1768628296000495136,19966848,-2.5128,1.78805,1.61059,-0.204767,-0.396967,0.608255,0.01823,0.0503516,-0.0114368,-0.9985,-0.0377556,-0.100303,0.0248023,1200,1900,1257,1257
1768628296020368384,19873248,-2.5128,1.78805,1.61059,-0.204767,-0.396967,0.608255,0.0370435,0.0486206,-0.0154022,-0.998011,-0.0758594,-0.0960544,0.0345112,1200,1900,1257,1257


通过图像观察：
第一次最低点时间戳：  1768628278911676288  1768628278960293408  1768628279011253184
第二次最低点时间戳：  1768628294240075296  1768628294401857248
加回180ms之后：
第一次最低点时间戳：  1768628279140293408  
第二次最低点时间戳：  1768628294420075296

计算出第一次时间戳相差： 1768628279460429216 - 1768628279140293408 = 320135808
计算出第二次时间戳相差： 1768628295960464960 - 1768628294420075296 = 320135808





第二次飞行
1768628789897769056,19935168,7.62707,7.76344,2.20535,0.137728,0.0212956,-1.18987,0.00313702,0.0262736,0.032236,-0.99913,-0.00458106,-0.0527281,-0.0643849,1609,1563,1502,1502
1768628789917750208,19981152,7.62707,7.76344,2.20535,0.137728,0.0212956,-1.18987,0.00313702,0.0262736,0.032236,-0.99913,-0.00458106,-0.0527281,-0.0643849,1609,1563,1502,1502
1768628789937841408,20091200,7.6319,7.75835,2.18093,0.111796,-0.277019,-0.237394,-0.0297551,0.0321663,0.0321499,-0.998522,0.0616494,-0.0623647,-0.0662964,1609,1563,1502,1502
1768628789957739072,19897664,7.63744,7.74599,2.17437,0.0890235,-0.365539,0.411767,-0.0271854,0.0234171,0.0285345,-0.998949,0.0557359,-0.0452489,-0.0583751,1609,1563,1502,1502
1768628789977883328,1768628789977883328,7.63744,7.74599,2.17437,0.0890235,-0.365539,0.411767,-0.0315304,0.0144081,0.0235561,-0.999121,0.0637512,-0.0273088,-0.0480157,1609,1563,1502,1502
1768628789997833472,19950144,7.63744,7.74599,2.17437,0.0890235,-0.365539,0.411767,-0.0303822,0.00850664,0.019135,-0.999319,0.0610942,-0.0158396,-0.0387754,1730,1900,1828,1828
1768628790017810048,19976576,7.64245,7.73201,2.18509,0.101118,-0.297843,0.669853,-0.0252645,-4.09101e-05,0.0147111,-0.999573,0.0505277,0.000825124,-0.0294119,1730,1900,1828,1828
1768628790037950528,20140480,7.64876,7.71913,2.205,0.156687,-0.330017,0.931418,-0.0252645,-4.09101e-05,0.0147111,-0.999573,0.0505277,0.000825124,-0.0294119,1730,1900,1828,1828
1768628790058002688,20052160,7.64876,7.71913,2.205,0.156687,-0.330017,0.931418,-0.0170648,-0.0091647,0.0102751,-0.99976,0.0339455,0.0186768,-0.0202375,1730,1900,1828,1828

最低点时间戳     1768628789957739072 1768628789977883328 1768628789997833472

图像最低点时间戳 1768628787781326880  下降 1768628788502727488  上升  1768628788909476672

相减 

*/