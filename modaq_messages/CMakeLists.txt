cmake_minimum_required(VERSION 3.8)
project(modaq_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Hmi.msg" 
  "msg/LjDo.msg" 
  "msg/Systemmsg.msg"
  "msg/T8ain.msg"
  "msg/T8ainstream.msg"
  "msg/T8dio.msg"
  "msg/Bagcontrol.msg"
  "msg/T8dac.msg"
  "msg/Ed582.msg"
  "msg/Buoy.msg"
  "msg/Timestatus.msg"
  DEPENDENCIES std_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)



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
