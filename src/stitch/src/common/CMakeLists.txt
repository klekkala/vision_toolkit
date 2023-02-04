set(LibName "3dobj_tracking_common")

add_library(${LibName} 
   CliParser.cpp 
   CliParser.h 
   ImageIO.cpp 
   ImageIO.h
)

target_include_directories( ${LibName} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../ )

target_link_libraries( ${LibName} LINK_PUBLIC
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
)

