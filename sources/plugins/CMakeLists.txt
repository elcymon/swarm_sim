# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/custom_msgs/robot_info
  ${CMAKE_CURRENT_BINARY_DIR}/m_nest
  ${CMAKE_CURRENT_BINARY_DIR}/mp_swarm1
  ${CMAKE_CURRENT_BINARY_DIR}/w_swarm1
  )
link_directories(${GAZEBO_LIBRARY_DIRS}
          ${CMAKE_CURRENT_BINARY_DIR}/custom_msgs/robot_info
          ${CMAKE_CURRENT_BINARY_DIR}/m_nest
          ${CMAKE_CURRENT_BINARY_DIR}/mp_swarm1
          ${CMAKE_CURRENT_BINARY_DIR}/w_swarm1
        )

add_subdirectory(custom_msgs/robot_info)
add_subdirectory(m_nest)
add_subdirectory(mp_swarm1)
add_subdirectory(w_swarm1)


add_library(nest_plugin SHARED m_nest/nest_plugin.cc )
target_link_libraries(nest_plugin custom_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(nest_plugin custom_msgs)

add_library(mp_swarm1 SHARED mp_swarm1/mp_model_push_split.cc)
target_link_libraries(mp_swarm1 custom_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(mp_swarm1 custom_msgs)


add_library(wp_swarm1 SHARED w_swarm1/wp_swarm1.cc)
target_link_libraries(wp_swarm1 ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(wp_swarm1 custom_msgs)
