find_package(gazebo REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)
link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_library(GimbalControllerPlugin SHARED GimbalControllerPlugin.cc)
target_link_libraries(GimbalControllerPlugin ${GAZEBO_LIBRARIES})

install(TARGETS
GimbalControllerPlugin
  DESTINATION ${PROJECT_SOURCE_DIR}/bin
)