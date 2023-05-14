# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS327A_BINARY_DIR}/hw5)
ADD_EXECUTABLE (hw5_p2 hw5_p2.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw5_p2 ${CS327A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_hw5_p2.urdf object.urdf DESTINATION ${APP_RESOURCE_DIR})

# copy plotting interface to output
FILE(COPY hw5_interfaces_plotter.html DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# copy in sai2-interfaces 
FILE(COPY ${SAI2-INTERFACES_UI_DIR} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# copy launch script
FILE(COPY launch_hw5_p2.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
