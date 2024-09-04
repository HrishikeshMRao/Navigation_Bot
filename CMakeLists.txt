cmake_minimum_required(VERSION 3.5)
project(navigation_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)

#target_link_libraries(Image_capture ${OpenCV_LIBS})
  
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

set(SOURCES1 /home/fiend/Navigation_Bot/dev_ws/src/Executables/ImageCapture.cpp)
# set(SOURCES2 /home/fiend/Navigation_Bot/dev_ws/src/Executables/follower.cpp)
set(SOURCES3 /home/fiend/Navigation_Bot/dev_ws/src/Executables/WebCam.cpp)
set(SOURCES4 /home/fiend/Navigation_Bot/dev_ws/src/Executables/test.cpp)

add_executable(ImageCapture ${SOURCES1})
ament_target_dependencies(ImageCapture rclcpp cv_bridge OpenCV std_msgs sensor_msgs image_transport geometry_msgs)
target_link_libraries(ImageCapture ${OpenCV_LIBS})

# add_executable(Motor_Driver_Node ${SOURCES2})
# ament_target_dependencies(Motor_Driver_Node rclcpp cv_bridge OpenCV std_msgs sensor_msgs image_transport geometry_msgs)
# target_link_libraries(Motor_Driver_Node ${OpenCV_LIBS})

# add_executable(webcam_publisher ${SOURCES3})
# ament_target_dependencies(webcam_publisher rclcpp cv_bridge OpenCV std_msgs sensor_msgs image_transport)
# target_link_libraries(webcam_publisher ${OpenCV_LIBS})

# add_executable(cv_bridge_test ${SOURCES4})
# ament_target_dependencies(cv_bridge_test rclcpp cv_bridge OpenCV std_msgs sensor_msgs image_transport)
# target_link_libraries(cv_bridge_test ${OpenCV_LIBS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds Maze
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    ImageCapture 
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
