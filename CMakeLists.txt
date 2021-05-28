cmake_minimum_required(VERSION 3.10)

project(3dobject_tracking)

find_package( OpenCV REQUIRED )
find_package( Boost 
   COMPONENTS 
      program_options 
      filesystem

   REQUIRED
)

include_directories( 
   ${OpenCV_INCLUDE_DIRS} 
   ${Boost_INCLUDE_DIRS}
)

add_subdirectory(src)
