cmake_minimum_required(VERSION 3.10)

project(3dobject_tracking)

find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )

add_subdirectory(src)