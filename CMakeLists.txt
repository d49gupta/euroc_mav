cmake_minimum_required(VERSION 3.8)
project(sensor_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-Wno-reorder)
add_compile_options(-Wno-unused-but-set-variable)
add_compile_options(-Wno-class-memaccess)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(DBoW3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

add_executable(imu
  src/csv_reader.cpp
  src/math_utils.cpp
  src/ahrs.cpp
  core/main_imu.cpp
)

add_executable(cam
  src/image_reader.cpp
  src/dataset_config.cpp
  src/math_utils.cpp
  src/vo.cpp
  core/main_cam.cpp
)

add_executable(vio
  src/csv_reader.cpp
  src/math_utils.cpp
  src/ahrs.cpp
  src/image_reader.cpp
  src/dataset_config.cpp
  src/vo.cpp
  src/vio.cpp
  core/main_vio.cpp
)

# Subscribes to the sim's sensor + ground-truth topics (see sim/); no
# fusion/estimation logic, just the data-in layer.
add_executable(sim_subscribers
  src/sim_subscribers.cpp
  src/sim_dataset_config.cpp
  src/math_utils.cpp
  src/vo.cpp
  core/main_sim_subscribers.cpp
)

target_link_libraries(imu
  Eigen3::Eigen
)

target_link_libraries(cam
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${DBoW3_LIBS}
  ${CERES_LIBRARIES}
)

target_link_libraries(vio
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${DBoW3_LIBS}
  ${CERES_LIBRARIES}
)

target_include_directories(imu PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/inc
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(cam PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/inc
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(vio PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/inc
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${DBoW3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

target_include_directories(sim_subscribers PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/inc
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(sim_subscribers
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

ament_target_dependencies(imu rclcpp std_msgs sensor_msgs nav_msgs)
ament_target_dependencies(cam rclcpp std_msgs sensor_msgs nav_msgs)
ament_target_dependencies(vio rclcpp std_msgs sensor_msgs nav_msgs)
ament_target_dependencies(sim_subscribers rclcpp std_msgs sensor_msgs nav_msgs cv_bridge message_filters)

install(TARGETS
  imu
  cam
  vio
  sim_subscribers
  DESTINATION lib/${PROJECT_NAME}
)

# Gazebo simulation assets (world/launch/bridge config); no lidar or
# vision processing code lives here -- see sim/README.md.
install(DIRECTORY
  sim/
  DESTINATION share/${PROJECT_NAME}/sim
)

ament_package()
