cmake_minimum_required(VERSION 3.8)
project(core_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Generate the interfaces
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GeofenceCompliance.srv"
  "srv/YoloImageDetect.srv" 
  "action/MoveTo.action"
  "srv/GridCreator.srv"
  "srv/MoveTo.srv"
  "action/PathPlanner.action"
  "msg/DetectedMsg.msg"
  "msg/YoloClassifiedObject.msg" 
  "msg/PointcloudDetectedObj.msg"
  "action/MoveToPickup.action"
  "srv/Pickup.srv"
  "srv/MoveArm.srv"
  "srv/MoveToObject.srv"
  "srv/Place.srv"
  "srv/PointcloudDetect.srv"
  "srv/LidarDetect.srv"
  "srv/LidarlikeFromCamera.srv"
  "srv/CameraDetect.srv"
  DEPENDENCIES nav_msgs
  DEPENDENCIES geometry_msgs 
  DEPENDENCIES std_msgs 
  DEPENDENCIES sensor_msgs
)

ament_package()


