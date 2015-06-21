######################################################################################
# Build for main library
######################################################################################

include_directories (
${CMAKE_CURRENT_SOURCE_DIR}
)


set(CMAKE_C_FLAGS "-Wall")

set(LIB_SRCS
CRForestEstimator.cpp
CRTree.cpp
gl_camera.cpp
)

set (HEADERS
gl_camera.hpp
CRForestEstimator.h  
CRTree.h 
CRForest.h 
freeglut.h
)

add_library (head_pose_fanelli SHARED ${LIB_SRCS})
add_library (head_pose_fanellistatic STATIC ${LIB_SRCS})
set_target_properties (head_pose_fanellistatic PROPERTIES OUTPUT_NAME head_pose_fanelli)

target_link_libraries(head_pose_fanelli OpenNI ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(head_pose_fanellistatic OpenNI ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${OpenCV_LIBRARIES})

#install library
install (TARGETS head_pose_fanelli LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install (TARGETS head_pose_fanellistatic ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install the header files
install (FILES ${HEADERS} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
