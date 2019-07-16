// C++
#include <string>
#include <sstream>
#include <array>

// Catch2
#include <catch2/catch.hpp>

// This repository
#include "tuple.hpp"

TEST_CASE("TupleTest -- sliceOne", "sliceOne") {
	auto tuple = std::make_tuple(1, true, 3);
	REQUIRE(   1 == std::tuple_size<decltype(dr::tuple_slice<0, 1>(tuple))>::value);
	REQUIRE(   1 == std::tuple_size<decltype(dr::tuple_slice<1, 1>(tuple))>::value);
	REQUIRE(   1 == std::tuple_size<decltype(dr::tuple_slice<2, 1>(tuple))>::value);
	REQUIRE(   1 == std::get<0>(dr::tuple_slice<0, 1>(tuple)));
	REQUIRE(true == std::get<0>(dr::tuple_slice<1, 1>(tuple)));
	REQUIRE(   3 == std::get<0>(dr::tuple_slice<2, 1>(tuple)));
}

TEST_CASE("TupleTest -- sliceTwo", "sliceTwo") {
	auto tuple = std::make_tuple(1, true, 3);
	REQUIRE(   2 == std::tuple_size<decltype(dr::tuple_slice<0, 2>(tuple))>::value);
	REQUIRE(   2 == std::tuple_size<decltype(dr::tuple_slice<1, 2>(tuple))>::value);
	REQUIRE(   1 == std::get<0>(dr::tuple_slice<0, 2>(tuple)));
	REQUIRE(true == std::get<1>(dr::tuple_slice<0, 2>(tuple)));
	REQUIRE(true == std::get<0>(dr::tuple_slice<1, 2>(tuple)));
	REQUIRE(   3 == std::get<1>(dr::tuple_slice<1, 2>(tuple)));
}

TEST_CASE("TupleTest -- sliceThree", "sliceThree") {
	auto tuple = std::make_tuple(1, true, 3);
	REQUIRE(   3 == std::tuple_size<decltype(dr::tuple_slice<0, 3>(tuple))>::value);
	REQUIRE(   1 == std::get<0>(dr::tuple_slice<0, 3>(tuple)));
	REQUIRE(true == std::get<1>(dr::tuple_slice<0, 3>(tuple)));
	REQUIRE(   3 == std::get<2>(dr::tuple_slice<0, 3>(tuple)));
}

TEST_CASE("TupleTest -- tail", "tail") {
	auto tuple = std::make_tuple(1, true, 3);
	REQUIRE(   2 == std::tuple_size<decltype(dr::tuple_tail(tuple))>::value);
	REQUIRE(true == std::get<0>(dr::tuple_tail(tuple)));
	REQUIRE(   3 == std::get<1>(dr::tuple_tail(tuple)));
}

TEST_CASE("TupleTest -- tailEmpty", "tailEmpty") {
	auto tuple = std::make_tuple(1);
	REQUIRE(0 == std::tuple_size<decltype(dr::tuple_tail(tuple))>::value);
}

/*
#include <gtest/gtest.h>

using namespace dr;

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

TEST(TupleTest, sliceOne) {
	auto tuple = std::make_tuple(1, true, 3);
	EXPECT_EQ(1,    std::tuple_size<decltype(tuple_slice<0, 1>(tuple))>::value);
	EXPECT_EQ(1,    std::tuple_size<decltype(tuple_slice<1, 1>(tuple))>::value);
	EXPECT_EQ(1,    std::tuple_size<decltype(tuple_slice<2, 1>(tuple))>::value);
	EXPECT_EQ(1,    std::get<0>(tuple_slice<0, 1>(tuple)));
	EXPECT_EQ(true, std::get<0>(tuple_slice<1, 1>(tuple)));
	EXPECT_EQ(3,    std::get<0>(tuple_slice<2, 1>(tuple)));
}

TEST(TupleTest, sliceTwo) {
	auto tuple = std::make_tuple(1, true, 3);
	EXPECT_EQ(2, std::tuple_size<decltype(tuple_slice<0, 2>(tuple))>::value);
	EXPECT_EQ(2, std::tuple_size<decltype(tuple_slice<1, 2>(tuple))>::value);
	EXPECT_EQ(1,    std::get<0>(tuple_slice<0, 2>(tuple)));
	EXPECT_EQ(true, std::get<1>(tuple_slice<0, 2>(tuple)));
	EXPECT_EQ(true, std::get<0>(tuple_slice<1, 2>(tuple)));
	EXPECT_EQ(3,    std::get<1>(tuple_slice<1, 2>(tuple)));
}

TEST(TupleTest, sliceThree) {
	auto tuple = std::make_tuple(1, true, 3);
	EXPECT_EQ(3, std::tuple_size<decltype(tuple_slice<0, 3>(tuple))>::value);
	EXPECT_EQ(1,    std::get<0>(tuple_slice<0, 3>(tuple)));
	EXPECT_EQ(true, std::get<1>(tuple_slice<0, 3>(tuple)));
	EXPECT_EQ(3,    std::get<2>(tuple_slice<0, 3>(tuple)));
}

TEST(TupleTest, tail) {
	auto tuple = std::make_tuple(1, true, 3);
	EXPECT_EQ(2,    std::tuple_size<decltype(tuple_tail(tuple))>::value);
	EXPECT_EQ(true, std::get<0>(tuple_tail(tuple)));
	EXPECT_EQ(3,    std::get<1>(tuple_tail(tuple)));
}

TEST(TupleTest, tailEmpty) {
	auto tuple = std::make_tuple(1);
	EXPECT_EQ(0, std::tuple_size<decltype(tuple_tail(tuple))>::value);
}
*/
