set(FOVIS_SRCS
depth_image.cpp
fast.cpp
feature_matcher.cpp
frame.cpp
gauss_pyramid.c
grid_filter.cpp
initial_homography_estimation.cpp
intensity_descriptor.cpp
internal_utils.cpp
motion_estimation.cpp
normalize_image.cpp
primesense_depth.cpp
pyramid_level.cpp
rectification.cpp
refine_feature_match.cpp
refine_motion_estimate.cpp
stereo_depth.cpp
stereo_frame.cpp
stereo_rectify.cpp
tictoc.cpp
visual_odometry.cpp
)

include_directories(
/usr/include/
${EIGEN_INCLUDE_DIRS}
${BOOST_INCLUDE_DIRS}
${OPENNI_INCLUDE_DIRS}
${IMAGEMAGICK_INCLUDE_DIRS}
${CMAKE_SOURCE_DIR}
${CMAKE_SOURCE_DIR}/fovis/
)

link_directories(
#${IMAGEMAGICK_LIBS}
${ARGTABLE_LIB}
)

add_library(pdt_fovis ${FOVIS_SRCS})
target_link_libraries(pdt_fovis -lX11 -lpthread -largtable2)

