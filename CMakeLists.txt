cmake_minimum_required(VERSION 2.8.3)
project(pid_server)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs)

add_service_files(FILES PID.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime roscpp std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/pid.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
