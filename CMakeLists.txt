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

add_executable(imu
  csv_reader.cpp
  math_utils.cpp
  ahrs.cpp
  main_imu.cpp
)

add_executable(cam
  image_reader.cpp
  main_cam.cpp
)

target_link_libraries(imu
  Eigen3::Eigen
)

target_link_libraries(cam
  ${OpenCV_LIBS}
)


target_include_directories(imu PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(cam PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)


install(TARGETS
  cam
  DESTINATION lib/${PROJECT_NAME}
)
