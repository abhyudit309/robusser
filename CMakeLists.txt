# - SAI2-URDF
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/robusser)
ADD_EXECUTABLE (controller_robusser_robot controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (eef_mobile_base_controller_robusser_robot eef_mobile_base_controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (eef_controller_robusser_robot eef_controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (mobile_base_controller_robusser_robot mobile_base_controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_robusser_robot simviz.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_dishwasher_robot dishwasher_controller.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_robusser_robot ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (eef_controller_robusser_robot ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (eef_mobile_base_controller_robusser_robot ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (mobile_base_controller_robusser_robot ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_robusser_robot ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_dishwasher_robot ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_robusser.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY robusser_robot.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY dishwasher_robot.urdf DESTINATION ${APP_RESOURCE_DIR})
