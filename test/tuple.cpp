#include <string>
#include <sstream>
#include <array>

#include <gtest/gtest.h>

#include "tuple.hpp"

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(TupleTest, sliceOne) { //NOLINT
	auto tuple = std::make_tuple(1, true, 3);
	EXPECT_EQ(1u,    std::tuple_size<decltype(tuple_slice<0, 1>(tuple))>::value);
	EXPECT_EQ(1u,    std::tuple_size<decltype(tuple_slice<1, 1>(tuple))>::value);
	EXPECT_EQ(1u,    std::tuple_size<decltype(tuple_slice<2, 1>(tuple))>::value);
	EXPECT_EQ(1,    std::get<0>(tuple_slice<0, 1>(tuple)));
	EXPECT_EQ(true, std::get<0>(tuple_slice<1, 1>(tuple)));
	EXPECT_EQ(3,    std::get<0>(tuple_slice<2, 1>(tuple)));
}

TEST(TupleTest, sliceTwo) { //NOLINT
	auto tuple = std::make_tuple(1, true, 3);
	EXPECT_EQ(2u, std::tuple_size<decltype(tuple_slice<0, 2>(tuple))>::value);
	EXPECT_EQ(2u, std::tuple_size<decltype(tuple_slice<1, 2>(tuple))>::value);
	EXPECT_EQ(1,    std::get<0>(tuple_slice<0, 2>(tuple)));
	EXPECT_EQ(true, std::get<1>(tuple_slice<0, 2>(tuple)));
	EXPECT_EQ(true, std::get<0>(tuple_slice<1, 2>(tuple)));
	EXPECT_EQ(3,    std::get<1>(tuple_slice<1, 2>(tuple)));
}

TEST(TupleTest, sliceThree) { //NOLINT
	auto tuple = std::make_tuple(1, true, 3);
	EXPECT_EQ(3u, std::tuple_size<decltype(tuple_slice<0, 3>(tuple))>::value);
	EXPECT_EQ(1,    std::get<0>(tuple_slice<0, 3>(tuple)));
	EXPECT_EQ(true, std::get<1>(tuple_slice<0, 3>(tuple)));
	EXPECT_EQ(3,    std::get<2>(tuple_slice<0, 3>(tuple)));
}

TEST(TupleTest, tail) { //NOLINT
	auto tuple = std::make_tuple(1, true, 3);
	EXPECT_EQ(2u,    std::tuple_size<decltype(tuple_tail(tuple))>::value);
	EXPECT_EQ(true, std::get<0>(tuple_tail(tuple)));
	EXPECT_EQ(3,    std::get<1>(tuple_tail(tuple)));
}

TEST(TupleTest, tailEmpty) { //NOLINT
	auto tuple = std::make_tuple(1);
	EXPECT_EQ(0u, std::tuple_size<decltype(tuple_tail(tuple))>::value);
}
} //namespace dr
