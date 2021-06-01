set(ProgramName "3dobj_tracking_pano")

add_executable(${ProgramName} 
   pano.cpp 
   CliParser.cpp 
   CliParser.h 
   ImageIO.cpp 
   ImageIO.h
)

target_link_libraries( ${ProgramName} LINK_PUBLIC
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
)
