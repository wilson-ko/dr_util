cmake_minimum_required(VERSION 2.8.3)
project(dr_util)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)
set(CMAKE_CXX_EXTENSIONS off)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(catkin REQUIRED COMPONENTS dr_cmake)
find_package(Boost REQUIRED COMPONENTS filesystem date_time)
include_directories(
	"include/${PROJECT_NAME}"
	SYSTEM "${Boost_INCLUDE_DIRS}"
)

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

dr_add_gtest(functional       test/functional.cpp)
dr_add_gtest(integer_sequence test/integer_sequence.cpp)
dr_add_gtest(tuple            test/tuple.cpp)
dr_add_gtest(bitwise          test/bitwise.cpp)
dr_add_gtest(button_filter    test/button_filter.cpp)
dr_add_gtest(geometry         test/geometry.cpp)
dr_add_gtest(polynomial       test/polynomial.cpp)
dr_add_gtest(voidt_t          test/void_t.cpp)

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
