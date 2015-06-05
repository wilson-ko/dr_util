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
	src/polynomial.cpp
	src/xmlrpc.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(record_robot_position src/record_robot_position.cpp)
target_link_libraries(record_robot_position ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_test_functional       test/functional.cpp)
catkin_add_gtest(${PROJECT_NAME}_test_integer_sequence test/integer_sequence.cpp)
catkin_add_gtest(${PROJECT_NAME}_test_tuple            test/tuple.cpp)
catkin_add_gtest(${PROJECT_NAME}_test_bitwise          test/bitwise.cpp)
catkin_add_gtest(${PROJECT_NAME}_test_button_filter    test/button_filter.cpp)
catkin_add_gtest(${PROJECT_NAME}_test_geometry         test/geometry.cpp)
catkin_add_gtest(${PROJECT_NAME}_test_polynomial       test/polynomial.cpp)
catkin_add_gtest(${PROJECT_NAME}_test_xmlrpc           test/xmlrpc.cpp)

target_link_libraries(${PROJECT_NAME}_test_polynomial ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_test_xmlrpc     ${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.hpp"
)
