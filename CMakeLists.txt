cmake_minimum_required(VERSION 3.10)

project(3dobject_tracking)

find_package( OpenCV REQUIRED )
find_package( Boost 
   COMPONENTS 
      program_options 
      filesystem

   REQUIRED
)
find_package( catkin 
   REQUIRED
   COMPONENTS
      roscpp
      sensor_msgs
      cv_bridge
      std_msgs
      image_transport
)

include_directories( 
   ${OpenCV_INCLUDE_DIRS} 
   ${Boost_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
