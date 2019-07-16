// C++
#include <string>
#include <sstream>
#include <array>

// Catch2
#include <catch2/catch.hpp>

// This repository
#include "functional.hpp"
#include "tuple.hpp"

TEST_CASE("FunctionalTest -- callHardcoded", "callHardcoded") {
	REQUIRE(true  == dr::call([] () { return true;  }, std::make_tuple()));
	REQUIRE(false == dr::call([] () { return false; }, std::make_tuple()));
}

TEST_CASE("FunctionalTest -- callInput", "callInput") {
	REQUIRE(true  == dr::call([] (bool input) { return input; }, std::make_tuple(true)));
	REQUIRE(false == dr::call([] (bool input) { return input; }, std::make_tuple(false)));
}

TEST_CASE("FunctionalTest -- callSum", "callSum") {
	REQUIRE( 3 == dr::call([] (int a, int b) { return a + b; }, std::make_tuple(1, 2)));
	REQUIRE(17 == dr::call([] (int a, int b) { return a + b; }, std::make_tuple(7, 10)));
}

/*
#include <gtest/gtest.h>

using namespace dr;

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

TEST(FunctionalTest, callHardcoded) {
	EXPECT_EQ(true,  call([] () { return true;  }, std::make_tuple()));
	EXPECT_EQ(false, call([] () { return false; }, std::make_tuple()));
}

TEST(FunctionalTest, callInput) {
	EXPECT_EQ(true,  call([] (bool input) { return input; }, std::make_tuple(true)));
	EXPECT_EQ(false, call([] (bool input) { return input; }, std::make_tuple(false)));
}

TEST(FunctionalTest, callSum) {
	EXPECT_EQ( 3, call([] (int a, int b) { return a + b; }, std::make_tuple(1, 2)));
	EXPECT_EQ(17, call([] (int a, int b) { return a + b; }, std::make_tuple(7, 10)));
}
*/
