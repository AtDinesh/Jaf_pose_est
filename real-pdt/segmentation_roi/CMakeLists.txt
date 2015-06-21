include_directories(
 /usr/include/ni # for OpenNI
 "${CMAKE_CURRENT_SOURCE_DIR}/include/pocketsphinx" # for pocketsphinx
 "${CMAKE_CURRENT_SOURCE_DIR}/include/sphinxbase" # for sphinxbase
  "${CMAKE_CURRENT_SOURCE_DIR}/src/real-pdt/" 
  include ${OpenCV_INCLUDE_DIRS} ${amppl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${freenect_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${USB_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
"${CMAKE_CURRENT_SOURCE_DIR}/include/"
"${CMAKE_CURRENT_SOURCE_DIR}/../"
${EIGEN3_INCLUDE_DIR}
)
set(SEGMENTATION_ROI_SRCS
ROI_SEG.cpp
SegmentationROI.cpp
SegmentedObj.cpp
)

include_directories(
/usr/include/
${EIGEN_INCLUDE_DIRS}
${BOOST_INCLUDE_DIRS}
${OPENNI_INCLUDE_DIRS}
${QT_INCLUDES}
${CMAKE_SOURCE_DIR}
${CMAKE_SOURCE_DIR}/segmentation_roi/
)

link_directories(
#${IMAGEMAGICK_LIBS}
${ARGTABLE_LIB}
)

add_library(segmentation_roi ${SEGMENTATION_ROI_SRCS})
target_link_libraries(segmentation_roi -lX11 -lpthread -largtable2 ${QT_LIBRARIES})

