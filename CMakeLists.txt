cmake_minimum_required(VERSION 2.8)
project(powered_caster_vehicle)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS    
    roscpp
    roslib    
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES powered_caster_vehicle
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

find_package(PkgConfig QUIET)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}   
  coppelia_src/remoteApi
  coppelia_src/include
)

add_definitions (-DNON_MATLAB_PARSING)
add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
#add_definitions (-DDO_NOT_USE_SHARED_MEMORY)

set (COPPELIA_SRC
  coppelia_src/remoteApi/extApi.c
  coppelia_src/remoteApi/extApiPlatform.c
  coppelia_src/common/shared_memory.c
)

set (SOURCES
	src/main.cpp
  src/vrep_bridge.cpp
  src/caster.cpp
  src/vehicle.cpp
  src/mobile_controller.cpp
  
  include/math_type_define.h
  include/linux_terminal_tool
  include/pcv_mtx_utils.h
)

add_executable(powered_caster_vehicle ${SOURCES} ${COPPELIA_SRC})

target_link_libraries(powered_caster_vehicle	
	pthread
	rt
)
# 

set(CMAKE_BUILD_TYPE RELEASE)
