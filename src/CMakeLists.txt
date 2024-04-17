find_package(gazebo REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_library(GimbalControllerPlugin SHARED GimbalControllerPlugin.cc)
target_link_libraries(GimbalControllerPlugin ${GAZEBO_LIBRARIES})
target_include_directories(GimbalControllerPlugin PRIVATE ../include)

add_library(ArduPilotPlugin SHARED ArduPilotPlugin.cc)
target_link_libraries(ArduPilotPlugin ${GAZEBO_LIBRARIES})
target_include_directories(ArduPilotPlugin PRIVATE ../include)

install(TARGETS
GimbalControllerPlugin
ArduPilotPlugin
  DESTINATION ${PROJECT_SOURCE_DIR}/bin
)