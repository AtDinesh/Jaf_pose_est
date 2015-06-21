######################################################################################
# Build for main library
######################################################################################

include_directories (
${CMAKE_CURRENT_SOURCE_DIR}
)

set(CMAKE_C_FLAGS "-Wall")

set(LIB_SRCS
_particle.cpp
_trackerpso.cpp
_observation.cpp
_dynamics.cpp
)

set (HEADERS
_state.h
_setting.h
_dynamics.h
_particle.h
_trackerpso.h
_observation.h
RandomWalk.h
WaveFormObs.h
HeadShoulderObs.h
HeadShoulderState.h
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
 
add_library (tpso_tracker_riddle SHARED ${LIB_SRCS})
add_library (tpso_trackerriddlestatic STATIC ${LIB_SRCS})
set_target_properties (tpso_trackerriddlestatic PROPERTIES OUTPUT_NAME tpso_tracker)

target_link_libraries(tpso_tracker_riddle ${OpenCV_LIBRARIES} -lrt -lpthread ${Boost_LIBRARIES}) #-lboost_thread)
target_link_libraries(tpso_trackerriddlestatic ${OpenCV_LIBRARIES} -lrt -lpthread ${Boost_LIBRARIES}) # -lboost_thread)

#install library
install (TARGETS tpso_tracker_riddle LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install (TARGETS tpso_trackerriddlestatic ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install the header files
install (FILES ${HEADERS} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
