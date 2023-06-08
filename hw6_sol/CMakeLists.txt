# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS327A_BINARY_DIR}/hw6_sol)
ADD_EXECUTABLE (hw6_solution hw6_solution.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw6_solution ${CS327A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_hw6.urdf DESTINATION ${APP_RESOURCE_DIR})