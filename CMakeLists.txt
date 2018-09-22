cmake_minimum_required(VERSION 3.8)
project(dr_util)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(catkin REQUIRED)
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
	src/environment.cpp
	src/expand.cpp
	src/file_system.cpp
	src/timestamp.cpp
	src/polynomial.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(${PROJECT_NAME}_test_bitwise          test/bitwise.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_button_filter    test/button_filter.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_environment      test/environment.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_expand           test/expand.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_functional       test/functional.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_geometry         test/geometry.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_integer_sequence test/integer_sequence.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_polynomial       test/polynomial.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_scope_guard      test/scope_guard.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_timestamp        test/timestamp.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_tuple            test/tuple.cpp)
	catkin_add_gtest(${PROJECT_NAME}_test_voidt_t          test/void_t.cpp)

	target_link_libraries(${PROJECT_NAME}_test_environment ${PROJECT_NAME})
	target_link_libraries(${PROJECT_NAME}_test_expand      ${PROJECT_NAME})
	target_link_libraries(${PROJECT_NAME}_test_polynomial  ${PROJECT_NAME})
	target_link_libraries(${PROJECT_NAME}_test_scope_guard ${PROJECT_NAME})
endif()

install(
	TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
	COMPONENT runtime
)

install(
	DIRECTORY "include/${PROJECT_NAME}/"
	DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
	COMPONENT development
)
