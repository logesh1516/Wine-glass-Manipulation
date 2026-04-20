cmake_minimum_required(VERSION 3.8)
project(isaacsim_winebot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  mtc_params # cmake target name for the parameter library
  config/mtc_params.yaml # path to input yaml file
)
add_executable(mtc_winebot src/mtc_winebot.cpp)
add_executable(mtc_winebot_glass src/mtc_winebot_glass.cpp)
target_include_directories(mtc_winebot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_winebot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(mtc_winebot_glass  mtc_params)
ament_target_dependencies(
  mtc_winebot
  "rclcpp"
  "moveit_task_constructor_core"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"

)
ament_target_dependencies(
  mtc_winebot_glass
  "rclcpp"
  "moveit_task_constructor_core"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
)
install(TARGETS mtc_winebot mtc_winebot_glass 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz meshes
  DESTINATION share/${PROJECT_NAME}
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
