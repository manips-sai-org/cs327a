# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS327A_BINARY_DIR}/hw1)
ADD_EXECUTABLE (hw1_control hw1_control.cpp ${CS327A_COMMON_SOURCE})
ADD_EXECUTABLE (hw1_viz hw1_viz.cpp ${CSCS327AA_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw1_control ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (hw1_viz ${CS327A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_hw1.urdf DESTINATION ${APP_RESOURCE_DIR})
