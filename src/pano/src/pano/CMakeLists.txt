set(ProgramName "3dobj_tracking_pano")

add_executable(${ProgramName} 
   pano.cpp 
)

target_link_libraries( ${ProgramName} LINK_PUBLIC
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
   3dobj_tracking_common
   ${catkin_LIBRARIES}
)
