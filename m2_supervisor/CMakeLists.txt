cmake_minimum_required(VERSION 3.8)
project(m2_supervisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(CURL REQUIRED)
find_package(modaq_messages REQUIRED)
find_package(rosbag2_cpp REQUIRED)



include_directories(include ${CMAKE_CURRENT_SOURCE_DIR}/include/m2_supervisor)
add_executable(m2_supervisor_node src/m2_supervisor_node.cpp src/EmailSender.cpp src/Logger.cpp src/Snoozer.cpp src/BagAnalyzer.cpp src/SystemStats.cpp)
ament_target_dependencies(m2_supervisor_node rclcpp std_msgs modaq_messages rosbag2_cpp)

##add_executable(bagAnalyzer src/BagAnalyzer.cpp)
##ament_target_dependencies(bagAnalyzer rosbag2_cpp  )

target_link_libraries(m2_supervisor_node "${cpp_typesupport_target}" curl)

ament_export_dependencies()

install(TARGETS
  m2_supervisor_node
  #bagAnalyzer
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
