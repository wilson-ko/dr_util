cmake_minimum_required(VERSION 2.8.3)
project(dr_util)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cmake_modules
)

find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES process_manager
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
)

add_library(process_manager src/process_manager.cpp)
target_link_libraries(process_manager ${catkin_LIBRARIES})
