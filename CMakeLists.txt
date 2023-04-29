
cmake_minimum_required (VERSION 2.8.12)
project (CS327A)
SET(CMAKE_CXX_FLAGS "-std=c++14 -I/usr/include -I/usr/local/include")

SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_BUILD_TYPE "Release")

# --------------------------------------------
# set common source
# --------------------------------------------
set (CS327A_COMMON_SOURCE
	# ${PROJECT_SOURCE_DIR}/src/redis/RedisClient.h
	${PROJECT_SOURCE_DIR}/src/timer/LoopTimer.h
	${PROJECT_SOURCE_DIR}/src/force_sensor/ForceSensorSim.h
	${PROJECT_SOURCE_DIR}/src/force_sensor/ForceSensorDisplay.h)
include_directories(${PROJECT_SOURCE_DIR}/src)
# include_directories(/usr/local/include) #uncomment if having problems with glfw library in mac

# --------------------------------------------
# set common dependencies
# --------------------------------------------

# - eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
	find_package(Eigen3 QUIET)
	# TODO: Find automatic way to find for Ubuntu 14.04
	if(NOT EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
	endif()
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

# - CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
link_directories(${CHAI3D_LIBRARY_DIRS})

# - SAI2-SIMULATION. 
find_package(SAI2-SIMULATION REQUIRED)
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

# - SAI2-MODEL
find_package(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})
add_definitions(${SAI2-MODEL_DEFINITIONS})

# - SAI2-GRAPHICS
find_package(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI2-GRAPHICS_DEFINITIONS})

# - SAI2-URDF
find_package(SAI2-URDF REQUIRED)
include_directories(${SAI2-URDF_INCLUDE_DIRS})
add_definitions(${SAI2-URDF_DEFINITIONS})

# - SAI2-INTERFACES
find_package(SAI2-INTERFACES REQUIRED)

# - glfw3
find_package(glfw3 QUIET)
find_library(GLFW_LIBRARY glfw)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# - jsoncpp
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
	find_library(JSONCPP_LIBRARY jsoncpp)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(JSONCPP jsoncpp)
	include_directories(${JSONCPP_INCLUDE_DIRS})
	link_directories(${JSONCPP_LIBRARY_DIRS})
	set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
endif ()

# --------------------------------------------
# set common libraries
# --------------------------------------------
set(CS327A_COMMON_LIBRARIES
	${CHAI3D_LIBARIES}
	${SAI2-COMMON_LIBRARIES}
	${SAI2-GRAPHICS_LIBRARIES}
	${SAI2-MODEL_LIBRARIES}
	${SAI2-SIMULATION_LIBRARIES}
	${SAI2-URDF_LIBRARIES}
	${HIREDIS_LIBRARY}
	${GLFW_LIBRARY}
	${JSONCPP_LIBRARY}
	)

# --------------------------------------------
# add apps and subdirectories
# --------------------------------------------
set (CS327A_BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(hw1)
add_subdirectory(hw2)
add_subdirectory(hw3)