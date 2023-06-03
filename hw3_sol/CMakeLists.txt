# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS327A_BINARY_DIR}/hw3_sol)
ADD_EXECUTABLE (hw3_p2_simviz_solution hw3_p2_simviz_solution.cpp)
ADD_EXECUTABLE (hw3_p2_controller_solution hw3_p2_controller_solution.cpp)
ADD_EXECUTABLE (hw3_p3_simviz_solution hw3_p3_simviz_solution.cpp)
ADD_EXECUTABLE (hw3_p3_controller_solution hw3_p3_controller_solution.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw3_p2_simviz_solution ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (hw3_p2_controller_solution ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (hw3_p3_simviz_solution ${CS327A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (hw3_p3_controller_solution ${CS327A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_hw3_p2.urdf world_hw3_p3.urdf DESTINATION ${APP_RESOURCE_DIR})

# copy plotting interface to output
FILE(COPY p3_interfaces_plotter.html DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# copy in sai2-interfaces 
FILE(COPY ${SAI2-INTERFACES_UI_DIR} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# copy launch scripts
FILE(COPY launch_p2_solution.sh launch_p3_solution.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
