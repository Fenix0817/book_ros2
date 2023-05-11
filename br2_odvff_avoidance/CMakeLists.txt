cmake_minimum_required(VERSION 3.5)
project(br2_odvff_avoidance)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(br2_tracking_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  br2_tracking_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  vision_msgs
  control_msgs
  image_transport
  cv_bridge
  OpenCV
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/br2_odvff_avoidance/AvoidanceNode.cpp
  src/br2_odvff_avoidance/ObjectDetector.cpp
  src/br2_odvff_avoidance/HeadController.cpp
  src/br2_odvff_avoidance/PIDController.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(avoidance_odvff src/avoidance_odvff_main.cpp)
ament_target_dependencies(avoidance_odvff ${dependencies})
target_link_libraries(avoidance_odvff ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  avoidance_odvff
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
