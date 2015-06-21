######################################################################################
# Build for main library
######################################################################################

include_directories (
${CMAKE_CURRENT_SOURCE_DIR}
)


set(CMAKE_C_FLAGS "-Wall")

set(LIB_SRCS
kinectInterface.cpp
kinectException.cpp
viewerOpenCV.cpp
)

set (HEADERS
kinectInterface.h
kinectException.h
viewerOpenCV.h
)

include_directories(
 /usr/include/ni # for OpenNI
 )
 
add_library (kinect_interface SHARED ${LIB_SRCS})
add_library (kinect_interfacestatic STATIC ${LIB_SRCS})
set_target_properties (kinect_interfacestatic PROPERTIES OUTPUT_NAME kinect_interface)

target_link_libraries(kinect_interface OpenNI ${OpenCV_LIBRARIES})
target_link_libraries(kinect_interfacestatic OpenNI ${OpenCV_LIBRARIES})

#install library
install (TARGETS kinect_interface LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install (TARGETS kinect_interfacestatic ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install the header files
install (FILES ${HEADERS} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
