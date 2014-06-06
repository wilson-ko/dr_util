cmake_minimum_required(VERSION 2.8.3)
project(dr_util)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules)
find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_spawner process_manager
  DEPENDS TinyXML
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
)

add_library(process_manager src/process_manager.cpp)
add_library(gazebo_spawner src/gazebo_spawner.cpp)
target_link_libraries(gazebo_spawner ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
