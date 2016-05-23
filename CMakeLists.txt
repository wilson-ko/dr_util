cmake_minimum_required(VERSION 2.8.3)
project(dr_util)

find_package(catkin REQUIRED COMPONENTS
	dr_base
	dr_log
	dr_param
	roscpp
	roslib
	tf
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS dr_log dr_param
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
	src/resource.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

dr_add_gtest(functional       test/functional.cpp)
dr_add_gtest(integer_sequence test/integer_sequence.cpp)
dr_add_gtest(tuple            test/tuple.cpp)
dr_add_gtest(bitwise          test/bitwise.cpp)
dr_add_gtest(button_filter    test/button_filter.cpp)
dr_add_gtest(geometry         test/geometry.cpp)
dr_add_gtest(polynomial       test/polynomial.cpp)
dr_add_gtest(resource         test/resource.cpp)

target_link_libraries(${PROJECT_NAME}_test_polynomial ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_test_resource   ${PROJECT_NAME} ${catkin_LIBRARIES})

install(
	TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
	DIRECTORY "include/${PROJECT_NAME}/"
	DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)
