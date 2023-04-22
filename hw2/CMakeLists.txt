# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS327A_BINARY_DIR}/hw2)
ADD_EXECUTABLE (hw2_simviz hw2_simviz.cpp)
ADD_EXECUTABLE (hw2_controller hw2_controller.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw2_simviz ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (hw2_controller ${CS327A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_hw2.urdf DESTINATION ${APP_RESOURCE_DIR})

# copy launch scripts
FILE(COPY launch.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})