cmake_minimum_required(VERSION 2.8.3)
project(dr_util)

find_package(catkin REQUIRED COMPONENTS
	dr_base
)

find_package(Boost REQUIRED COMPONENTS filesystem date_time)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	INCLUDE_DIRS include
	DEPENDS Boost
)

add_library(${PROJECT_NAME}
	src/file_system.cpp
	src/timestamp.cpp
	src/polynomial.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(${PROJECT_NAME}
	PRIVATE "include/${PROJECT_NAME}"
	SYSTEM PRIVATE ${Boost_INCLUDE_DIRS}
)

dr_add_gtest(functional       test/functional.cpp)
dr_add_gtest(integer_sequence test/integer_sequence.cpp)
dr_add_gtest(tuple            test/tuple.cpp)
dr_add_gtest(bitwise          test/bitwise.cpp)
dr_add_gtest(button_filter    test/button_filter.cpp)
dr_add_gtest(geometry         test/geometry.cpp)
dr_add_gtest(polynomial       test/polynomial.cpp)

target_link_libraries(${PROJECT_NAME}_test_polynomial ${PROJECT_NAME})

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
