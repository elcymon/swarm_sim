

# 2.8.8 required to use PROTOBUF_IMPORT_DIRS

cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

##PROTOBUF GENERATE SOURCE FILES
find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  custom_msgs/comm_data/comm_data.proto
  custom_msgs/robot_info/robot_info.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

###COMPILE PLUGINS
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
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/m_nest
  ${CMAKE_CURRENT_BINARY_DIR}/mp_swarm1
  ${CMAKE_CURRENT_BINARY_DIR}/w_swarm1
  )
link_directories(${GAZEBO_LIBRARY_DIRS}
          ${CMAKE_CURRENT_BINARY_DIR}
          ${CMAKE_CURRENT_BINARY_DIR}/m_nest
          ${CMAKE_CURRENT_BINARY_DIR}/mp_swarm1
          ${CMAKE_CURRENT_BINARY_DIR}/w_swarm1
        )

#add_subdirectory(custom_msgs/robot_info)
add_subdirectory(m_nest)
add_subdirectory(mp_swarm1)
add_subdirectory(w_swarm1)

#add_library(custom_msgs SHARED ${PROTO_SRCS})





add_library(nest_plugin SHARED ${PROTO_SRCS} m_nest/nest_plugin.cc )
target_link_libraries(nest_plugin 
              ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} 
              ${SDF_LIBRARIES} ${PROTOBUF_LIBRARY}
            )
#add_dependencies(nest_plugin custom_msgs)

add_library(mp_swarm1 SHARED mp_swarm1/mp_model_push_split.cc ${PROTO_SRCS})
target_link_libraries(mp_swarm1 
              ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} 
              ${SDF_LIBRARIES} ${PROTOBUF_LIBRARY}
            )
#add_dependencies(mp_swarm1 custom_msgs)


add_library(wp_swarm1 SHARED w_swarm1/wp_swarm1.cc ${PROTO_SRCS})
target_link_libraries(wp_swarm1 
              ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} 
              ${SDF_LIBRARIES} ${PROTOBUF_LIBRARY}
            )
#add_dependencies(wp_swarm1 custom_msgs)

