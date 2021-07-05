cmake_minimum_required(VERSION 2.8)
project(powered_caster_vehicle)



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


set (VREP_ROOT
    $ENV{HOME}/Downloads/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04/
    CACHE "VRep root dir" STRING)

include_directories(
  ${catkin_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}   
  ${VREP_ROOT}/programming/include 
  ${VREP_ROOT}/programming/remoteApi
)


include_directories (${VREP_PATH}/remoteApi ${VREP_PATH}/include include)

add_definitions (-DNON_MATLAB_PARSING)
add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
#add_definitions (-DDO_NOT_USE_SHARED_MEMORY)

# set (HEADS
#   # include/pcv/controller.h
#   include/powered_caster_vehicle/vrep_bridge.h

#   # include/utils/math_type_define.h
#   include/utils/linux_terminal_tool
# )

set (SOURCES
	src/main.cpp
  src/vrep_bridge.cpp
  src/caster.cpp
  src/vehicle.cpp
  src/mobile_controller.cpp
  
  include/math_type_define.h
  include/linux_terminal_tool
  include/pcv_mtx_utils.h

	${VREP_ROOT}/programming/remoteApi/extApi.c
	${VREP_ROOT}/programming/remoteApi/extApiPlatform.c
	${VREP_ROOT}/programming/common/shared_memory.c
)

add_executable(powered_caster_vehicle ${SOURCES})

target_link_libraries(powered_caster_vehicle	
	pthread
	rt
)
# 


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE RELEASE)
