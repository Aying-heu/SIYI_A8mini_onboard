cmake_minimum_required(VERSION 3.5)
project(A8mini)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 开启最高级优化（核心）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
# 2. 针对当前CPU架构优化（可选，进一步提速）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
# 3. 开启多线程编译（可选，编译速度快，不影响运行速度）
set(CMAKE_BUILD_PARALLEL_LEVEL 8)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    add_compile_options(-march=armv8-a)
endif()

# 添加onnxruntime库路径
# link_directories(/home/bsa/environments/onnxruntime/build/Linux/Release)
# link_directories(/home/bsa/environments/onnxruntime/build/Linux/RelWithDebInfo)
# include_directories(/home/bsa//environments/onnxruntime/onnxruntime/core/providers/cuda)
# include_directories(/home/bsa//environments/onnxruntime/include/onnxruntime/core/session/)
# include_directories(/usr/local/include/onnxruntime)
# include_directories(/usr/local/include/onnxruntime/core/session)



# 添加opencv库路径
include_directories(/usr/local/include/opencv4/opencv2/)
include_directories(/usr/local/include/opencv4/)
link_directories(/usr/local/lib)

find_path(ZBAR_INCLUDE_DIR NAMES zbar.h)
# 查找zbar库文件（Linux下通常为libzbar.so，Windows为zbar.lib）
find_library(ZBAR_LIBRARY NAMES zbar)

# 检查zbar是否找到
if(NOT ZBAR_INCLUDE_DIR OR NOT ZBAR_LIBRARY)
    message(FATAL_ERROR "ZBar not found! Please install libzbar-dev (Linux) or ZBar SDK (Windows).")
endif()

set(ONNXRUNTIME_HOME /home/bsa/environments/onnxruntime)
include_directories(${ONNXRUNTIME_HOME}/include)
include_directories(${ONNXRUNTIME_HOME}/include/onnxruntime/core/session)
link_directories(${ONNXRUNTIME_HOME}/lib)

set(ONNX /home/bsa/environments/onnxruntime/)
set(ONNX_MORE1 /usr/local/include/onnxruntime/core/session/)
set(ONNX_MORE2 /home/bsa/environments/onnxruntime/build/Linux/RelWithDebInfo)


find_package(  
    OpenCV
    REQUIRED
    COMPONENTS
    core
    imgproc
    imgcodecs
    highgui
    dnn
    aruco
    cudaimgproc  # 关键组件
    cudaarithm   # 通常需要
    cudawarping  # 包含 resize 实现
)

message(STATUS "Found OpenCV: ${OpenCV_VERSION}")
# 检查OpenCV库
if(OpenCV_FOUND)
    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
    
    # 检查并移除可能缺失的模块
    set(OPENCV_LIBS_TO_USE ${OpenCV_LIBS})
    if(NOT TARGET opencv_hdf)
        list(REMOVE_ITEM OPENCV_LIBS_TO_USE opencv_hdf)
        message(STATUS "Removed opencv_hdf from link libraries (not found)")
    endif()
    if(NOT TARGET opencv_viz)
        list(REMOVE_ITEM OPENCV_LIBS_TO_USE opencv_viz)
        message(STATUS "Removed opencv_viz from link libraries (not found)")
    endif()
    
    # 验证 CUDA 支持
    if(OpenCV_CUDA_VERSION)
        message(STATUS "OpenCV CUDA support: YES (CUDA ${OpenCV_CUDA_VERSION})")
    else()
        message(WARNING "OpenCV compiled without CUDA support! Performance will be degraded")
    endif()
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

# 查找其他依赖
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4") # 指向你的OpenCVConfig.cmake所在目录
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

include_directories(${EIGEN3_INCLUDE_DIRS})


# 查找zbar库（二维码解析库）
# 查找zbar头文件目录（通常包含zbar.h）
find_path(ZBAR_INCLUDE_DIR NAMES zbar.h)
# 查找zbar库文件（Linux下通常为libzbar.so，Windows为zbar.lib）
find_library(ZBAR_LIBRARY NAMES zbar)

# 检查zbar是否找到
if(NOT ZBAR_INCLUDE_DIR OR NOT ZBAR_LIBRARY)
    message(FATAL_ERROR "ZBar not found! Please install libzbar-dev (Linux) or ZBar SDK (Windows).")
endif()


include_directories(
  include/ 
  /usr/local/include/opencv4/
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  ${prometheus_msgs_INCLUDE_DIRS}
  ${mavros_INCLUDE_DIRS}
  ${GEOGRAPHICLIB_INCLUDE_DIR}
)

# 顶部：先找到 ROS/ament 等（你已有）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(prometheus_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  # 推荐使用 Eigen3::Eigen
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui dnn)

# ONNX Runtime（保持你当前的 find_path/find_library 逻辑，确保变量被设置）
find_path(ONNXRUNTIME_INCLUDE_DIR NAMES onnxruntime_cxx_api.h PATHS /home/bsa/environments/onnxruntime/include/onnxruntime/core/session/ /usr/local/include/onnxruntime)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime PATHS /home/bsa/environments/onnxruntime/build/Linux/Release /usr/local/lib)


# 目标：get_img_uav_gimbal
file(GLOB MODULES_FILES ${CMAKE_SOURCE_DIR}/modules/*.cpp)


add_executable(main_multi_thread src/main_multi_thread.cpp ${MODULES_FILES})
# 包含目录（只给这个 target）
target_include_directories(main_multi_thread PRIVATE
  ${ZBAR_LIBRARIES}
  ${ZBAR_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}        # 或直接用 Eigen3::Eigen
)

# 链接第三方库（OpenCV, ONNXRuntime, pthread 等）
target_link_libraries(main_multi_thread
  ${ZBAR_LIBRARY}
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIBRARY}
  pthread
  dl
)
# 用 ament 连接 ROS 依赖（只写 ament 的包名）
ament_target_dependencies(main_multi_thread
  rclcpp
  sensor_msgs
  # cv_bridge
  image_transport
  mavros_msgs
  prometheus_msgs
)





add_executable(Confirm_delay_time src/Confirm_delay_time.cpp ${MODULES_FILES})
# 包含目录（只给这个 target）
target_include_directories(Confirm_delay_time PRIVATE
  ${ZBAR_LIBRARIES}
  ${ZBAR_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}        # 或直接用 Eigen3::Eigen
)

# 链接第三方库（OpenCV, ONNXRuntime, pthread 等）
target_link_libraries(Confirm_delay_time
  ${ZBAR_LIBRARY}
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIBRARY}
  pthread
  dl
)

# 用 ament 连接 ROS 依赖（只写 ament 的包名）
ament_target_dependencies(Confirm_delay_time
  rclcpp
  sensor_msgs
  # cv_bridge
  image_transport
  mavros_msgs
  prometheus_msgs
)




add_executable(Confirm_delay_time2 src/Confirm_delay_time2.cpp ${MODULES_FILES})
# 包含目录（只给这个 target）
target_include_directories(Confirm_delay_time2 PRIVATE
  ${ZBAR_LIBRARIES}
  ${ZBAR_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}        # 或直接用 Eigen3::Eigen
)

# 链接第三方库（OpenCV, ONNXRuntime, pthread 等）
target_link_libraries(Confirm_delay_time2
  ${ZBAR_LIBRARY}
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIBRARY}
  pthread
  dl
)

# 用 ament 连接 ROS 依赖（只写 ament 的包名）
ament_target_dependencies(Confirm_delay_time2
  rclcpp
  sensor_msgs
  # cv_bridge
  image_transport
  mavros_msgs
  prometheus_msgs
)





add_executable(cam_pipeline test/cam_pipeline.cpp)
# 包含目录（只给这个 target）
target_include_directories(cam_pipeline PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

# 链接第三方库（OpenCV, ONNXRuntime, pthread 等）
target_link_libraries(cam_pipeline
  ${OpenCV_LIBS}
)




add_executable(test_gimbal test/test_gimbal.cpp)

target_include_directories(test_gimbal PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(test_gimbal
  ${OpenCV_LIBS}
)




add_executable(test_kalman test/test_kalman.cpp modules/target_state_Filter.cpp)
target_include_directories(test_kalman PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} 
)
target_link_libraries(test_kalman
  ${OpenCV_LIBS}
)
ament_target_dependencies(test_kalman
  rclcpp
  sensor_msgs
  # cv_bridge
  image_transport
  mavros_msgs
  prometheus_msgs
)





find_package(Python3 COMPONENTS Development REQUIRED)
set(PYTHON_INCLUDE_DIR "/usr/include/python3.9")
set(PYTHON_LIBRARY "/usr/lib/x86_64-linux-gnu/libpython3.9.so")

# 3. 找到NumPy的头文件路径
execute_process(
    COMMAND python3 -c "import numpy; print(numpy.get_include())"
    OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# add_executable(test_uav_tracking test/Kalman.cpp src/TargetTracker.cpp)
# target_include_directories(test_uav_tracking PRIVATE
#   ${Python3_LIBRARIES}
#   ${ZBAR_LIBRARIES}
#   ${ZBAR_INCLUDE_DIR}
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${OpenCV_INCLUDE_DIRS}
#   ${ONNXRUNTIME_INCLUDE_DIR}
#   ${EIGEN3_INCLUDE_DIR}        # 或直接用 Eigen3::Eigen
#   Python3::Python
# )


# add_executable(test_uav_tracking test/Kalman.cpp src/TargetTracker.cpp)
# target_include_directories(test_uav_tracking PRIVATE
#     ${ZBAR_LIBRARIES}
#     ${CMAKE_CURRENT_SOURCE_DIR}  # 璁╃紪璇戝櫒鎵惧埌module.hpp
#     ${OpenCV_INCLUDE_DIRS}
#     ${ZBAR_INCLUDE_DIR}
#     ${ONNXRUNTIME_INCLUDE_DIR}
#     ${CUDAToolkit_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIRS}
# )
# target_link_libraries(test_uav_tracking PRIVATE
#     ${OpenCV_LIBS}
#     ${ZBAR_LIBRARY}
#     ${ONNXRUNTIME_LIBRARY}
#     pthread       # 可选：系统线程库（部分环境需要链接）
#     dl
#     Python3::Python
# )