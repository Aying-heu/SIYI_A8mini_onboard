# chmod 777 /dev/ttyTHS0 

# ros2 launch mavros px4.launch

# ros2 launch prometheus_uav_control uav_control_main_outdoor_launch.py

# ros2 run prometheus_uav_control uav_command_pub

# /home/bsa/SIYI_A8mini/build/get_img_uav_gimbal


# 1. 给串口赋权限（如需sudo则加，建议配置免密避免输密码）
xterm -title "串口权限配置" -e "sudo chmod 777 /dev/ttyTHS0; bash" &
sleep 5

# 2. 启动mavros px4（ros2环境需已加载，如未自动加载则先source）
xterm -title "MAVROS PX4" -e "ros2 launch mavros px4.launch" &
sleep 1

# 3. 启动prometheus室外控制主节点
xterm -title "Prometheus UAV Control" -e "ros2 launch prometheus_uav_control uav_control_main_outdoor_launch.py" &
sleep 1

# 4. 运行指令发布节点
xterm -title "UAV Command Pub" -e "ros2 run prometheus_uav_control uav_command_pub" &
sleep 8

# 5. 运行思翼云台取图程序
xterm -title "SIYI A8mini Get Img" -e "/home/bsa/SIYI_A8mini_onboard/build/main_multi_thread; bash" &
~                                                                                                              