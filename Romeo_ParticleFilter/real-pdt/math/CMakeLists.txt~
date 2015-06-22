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

set(MATH_SRCS
Math.cpp
)

include_directories(
/usr/include/
${EIGEN_INCLUDE_DIRS}
${BOOST_INCLUDE_DIRS}
${OPENNI_INCLUDE_DIRS}
${IMAGEMAGICK_INCLUDE_DIRS}
${CMAKE_SOURCE_DIR}
${CMAKE_SOURCE_DIR}/math/
)

link_directories(
#${IMAGEMAGICK_LIBS}
${ARGTABLE_LIB}
)

add_library(pdt_lmath ${MATH_SRCS})
target_link_libraries(pdt_lmath -lX11 -lpthread -largtable2)

