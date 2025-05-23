message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME}_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread -pg")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

include_directories(${CMAKE_SOURCE_DIR})
include_directories ("/usr/include/opencv4/")
include_directories(${EIGEN3_INCLUDE_DIRS})

set(SRC ${SRC} 
  docking_pose_estimator_ros2.cc)
  
add_executable(docking_pose_estimator_node docking_pose_estimator_node.cc ${SRC})
target_link_libraries(docking_pose_estimator_node docking_pose_estimator_core ${EIGEN3_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(docking_pose_estimator_node rclcpp sensor_msgs geometry_msgs nav_msgs  )
install(TARGETS docking_pose_estimator_node
  DESTINATION lib/${PROJECT_NAME}) # install executable
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}) # install launch files
