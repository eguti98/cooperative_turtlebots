cmake_minimum_required(VERSION 3.0.2)
project(dekf_sensor_fusion)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs geometry_msgs)

add_message_files(
  FILES
  globalCovariance.msg
)

add_service_files(FILES SrvCov.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dekf_sensor_fusion
  CATKIN_DEPENDS message_generation roscpp std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME}
#  src/dekf_sensor_fusion.cpp
#)

add_executable(dekf_sensor_fusion src/dekf_sensor_fusion.cpp)

add_dependencies(dekf_sensor_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dekf_sensor_fusion ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
