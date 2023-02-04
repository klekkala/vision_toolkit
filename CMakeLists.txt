cmake_minimum_required(VERSION 3.10)

project(vision_toolkit)

find_package( OpenCV 4.2.0 REQUIRED )
find_package( Boost 
   COMPONENTS 
      program_options 
      filesystem

   REQUIRED
)

find_package( catkin REQUIRED
    COMPONENTS
        roscpp
        cv_bridge
        sensor_msgs
        std_msgs
        image_transport
        rosbag
)
find_package(PCL 1.10 REQUIRED
  COMPONENTS
    common
    tracking
    filters
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


include_directories( 
   ${OpenCV_INCLUDE_DIRS} 
   ${Boost_INCLUDE_DIRS}
   ${PCL_INCLUDE_DIRS}
)

