#include <string>
#include <sstream>
#include <array>

#include <gtest/gtest.h>

#include "tuple.hpp"

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
