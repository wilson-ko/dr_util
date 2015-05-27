cmake_minimum_required(VERSION 2.8.3)
project(dr_util)

find_package(catkin REQUIRED COMPONENTS
	dr_base
	dr_log
	roscpp
	tf
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS dr_log
)

dr_include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/file_system.cpp
	src/node.cpp
	src/signal.cpp
	src/timestamp.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(record_robot_position src/record_robot_position.cpp)
target_link_libraries(record_robot_position ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_functional_test       test/functional.cpp)
catkin_add_gtest(${PROJECT_NAME}_integer_sequence_test test/integer_sequence.cpp)
catkin_add_gtest(${PROJECT_NAME}_tuple_test            test/tuple.cpp)
catkin_add_gtest(${PROJECT_NAME}_bitwise_test          test/bitwise.cpp)
catkin_add_gtest(${PROJECT_NAME}_button_filter_test    test/button_filter.cpp)
catkin_add_gtest(${PROJECT_NAME}_geometry_test         test/geometry.cpp)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.hpp"
)
