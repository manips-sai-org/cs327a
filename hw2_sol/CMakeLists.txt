# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS327A_BINARY_DIR}/hw2_sol)
ADD_EXECUTABLE (hw2_simviz_solution hw2_simviz_solution.cpp)
ADD_EXECUTABLE (hw2_controller_solution hw2_controller_solution.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw2_simviz_solution ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (hw2_controller_solution ${CS327A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_hw2.urdf DESTINATION ${APP_RESOURCE_DIR})

# copy launch scripts
FILE(COPY launch_solution.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
