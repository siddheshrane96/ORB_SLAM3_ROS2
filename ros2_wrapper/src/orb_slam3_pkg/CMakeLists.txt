cmake_minimum_required(VERSION 3.8)
project(orb_slam3_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ORB_SLAM3_ROS2 REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(orb_slam3_node src/orb_slam3_node.cpp)

target_include_directories(orb_slam3_node
  PRIVATE
    # ${OpenCV_INCLUDE_DIRS}
    /usr/local/include/ORB_SLAM3_ROS2/Thirdparty/Sophus # Ugly hack
)

ament_target_dependencies(orb_slam3_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  ORB_SLAM3_ROS2
)
# TODO: later on make Eigen PUBLIC inside ORB_SLAM3
# Note here how Keywords PUBLIC PRIVATE etc are not used, to get a plain signature.
target_link_libraries(orb_slam3_node 
  Eigen3::Eigen
  ORB_SLAM3_ROS2::ORB_SLAM3_ROS2 # bring public include paths
  ${OPENGL_LIBRARIES}
  GLEW::GLEW)

install(TARGETS orb_slam3_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------- Video publisher Node ---------------------
add_executable(video_publisher_node src/video_publisher.cpp)
ament_target_dependencies(video_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_include_directories(video_publisher_node
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(video_publisher_node
  ${OpenCV_LIBS}
)
install(TARGETS video_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Launch file install
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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
