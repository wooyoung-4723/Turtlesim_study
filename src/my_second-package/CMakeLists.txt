cmake_minimum_required(VERSION 3.8)
project(my_second-package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)


add_executable(dist_turtle_action_server src/dist_turtle_action_server.cpp)

add_executable(my_subscriber src/my_subscriber.cpp)
ament_target_dependencies(my_subscriber rclcpp turtlesim)

add_executable(my_publisher src/my_publisher.cpp)
ament_target_dependencies(my_publisher rclcpp geometry_msgs)


target_include_directories(dist_turtle_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dist_turtle_action_server PUBLIC c_std_99 cxx_std_17)

install(TARGETS
 dist_turtle_action_server
 my_subscriber
 my_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
