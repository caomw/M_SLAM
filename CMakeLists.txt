cmake_minimum_required(VERSION 2.8.3)
project(M_SLAM)


set( CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -O3 -pthread")


list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )


find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(G2O )
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES M_SLAM
CATKIN_DEPENDS
        cv_bridge
        image_transport
        pcl_ros
        roscpp
        rospy
        sensor_msgs
        std_msgs tf
DEPENDS system_lib
)


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        /usr/local/include # Location when using 'make system_install'
        /usr/include       # More usual location (e.g. when installing using a package)
        ${PROJECT_SOURCE_DIR}/Thirdparty/
)

link_directories(
        /usr/local/lib # Location when using 'make system_install'
        /usr/lib       # More usual location (e.g. when installing using a package)
)


add_executable(Img2Pc src/Img2Pc.cpp)



target_link_libraries(Img2Pc ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} )
