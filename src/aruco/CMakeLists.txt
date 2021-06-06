set(ProgramName "3dobj_tracking_aruco")

add_executable(${ProgramName} aruco_pose.cpp)

target_link_libraries( ${ProgramName} LINK_PUBLIC
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
   3dobj_tracking_common
   ${catkin_LIBRARIES}
)
