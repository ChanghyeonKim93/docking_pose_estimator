# cmake_minimum_required(VERSION 3.10)
# project(${CMAKE_PROJECT_NAME}_core)
message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME}_core)

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -march=native")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories ("/usr/include/opencv4/")
include_directories(${EIGEN3_INCLUDE_DIRS})

set(SRC ${SRC}
  docking_pose_estimator.cc
)

# set(CMAKE_INSTALL_PREFIX )
add_library(${CMAKE_PROJECT_NAME}_core STATIC ${SRC})
target_link_libraries(${CMAKE_PROJECT_NAME}_core ${EIGEN3_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(${CMAKE_PROJECT_NAME}_core 
  PUBLIC ${CMAKE_SOURCE_DIR} ${EIGEN3_INCLUDE_DIRS})
install(TARGETS ${CMAKE_PROJECT_NAME}_core  
  DESTINATION lib/${CMAKE_PROJECT_NAME}
  CONFIGURATIONS Release)
  
message("   -- [INFO   ]: ${CMAKE_PROJECT_NAME}_core is copied to {ROS2_WS}/install/${CMAKE_PROJECT_NAME}_core/" lib/${CMAKE_PROJECT_NAME})
message("   -- [INFO   ]: filename: lib${CMAKE_PROJECT_NAME}_core.a")

