
include_directories(
 /usr/include/ni # for OpenNI
 "${CMAKE_CURRENT_SOURCE_DIR}/include/pocketsphinx" # for pocketsphinx
 "${CMAKE_CURRENT_SOURCE_DIR}/include/sphinxbase" # for sphinxbase
  "${CMAKE_CURRENT_SOURCE_DIR}/src/real-pdt/" 
  include ${OpenCV_INCLUDE_DIRS} ${amppl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${freenect_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${USB_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
"${CMAKE_CURRENT_SOURCE_DIR}/include/"
"${CMAKE_CURRENT_SOURCE_DIR}"
${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_CXX_FLAGS "-O3")

#if( $ENV{ROBOT} STREQUAL "pr2" )
 #   set(ARGTABLE_LIB "/home/aamekonn/src-install/lib" )
# endif()

message ("argtable lib path : " ${ARGTABLE_LIB} "on host " $ENV{HOSTNAME})

add_subdirectory(fovis/)
add_subdirectory(math/)
add_subdirectory(segmentation_roi/)

set(PDT_CORE_SRCS
AncillaryMethods.cpp
CPoint.cpp
depthdetector.cpp
depthdetector_lm.cpp
depthdetector_lm_seg.cpp
depthdetector_seg.cpp
Detections.cpp
detector.cpp
detector_seg.cpp
EKalman.cpp
FrameInlier.cpp
Globals.cpp
groundplaneestimator.cpp
Hypo.cpp
Kalman.cpp
KConnectedComponentLabeler.cpp
kinectgrabber.cpp
MDL.cpp
pointcloud.cpp
ROI.cpp
Tracker.cpp
Visualization.cpp
config_file/ConfigFile.cpp
camera/Camera.cpp
)

link_directories(
${ARGTABLE_LIB}
${CMAKE_BINARY_DIR}/fovis/
${CMAKE_BINARY_DIR}/math/
${CMAKE_BINARY_DIR}/segmentation_roi/
)

add_library(pdt_real_pdt ${PDT_CORE_SRCS})
target_link_libraries(pdt_real_pdt -lX11 -lpthread -largtable2 pdt_fovis pdt_lmath pdt_segmentation_roi)
