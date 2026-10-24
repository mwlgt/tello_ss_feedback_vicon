cmake_minimum_required(VERSION 3.8)
project(tello_feedback_vicon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(controller src/controller.cpp)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(pController src/pController.cpp)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(spiral src/spiral_pub.cpp)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(steps src/steps.cpp)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_target_dependencies(controller rclcpp std_msgs geometry_msgs)


ament_target_dependencies(pController rclcpp std_msgs geometry_msgs)

ament_target_dependencies(spiral rclcpp std_msgs geometry_msgs)

ament_target_dependencies(steps rclcpp std_msgs geometry_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  DEPENDENCIES geometry_msgs)

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pController
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS spiral
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS steps
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/tello_wrapper.py
  DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
