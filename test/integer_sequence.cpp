#include <string>
#include <sstream>
#include <array>

#include <gtest/gtest.h>

#include "integer_sequence.hpp"

using namespace dr;

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

template<typename T>
T sum(integer_sequence<T>) {
	return 0;
}

template<typename T, T first, T... I>
T sum(integer_sequence<T, first, I...>) {
	return first + sum(integer_sequence<T, I...>{});
}

TEST(IntegerSequenceTest, makeIntegerSequence) {
	EXPECT_EQ(3, sum(make_integer_sequence<int, 3>{}));
	EXPECT_EQ(3, (make_integer_sequence<int, 3>{}).size());
}

TEST(IntegerSequenceTest, makeIntegerSlice) {
	EXPECT_EQ(12, sum(make_integer_slice<int, 3, 6>{}));
	EXPECT_EQ(3, (make_integer_slice<int, 3, 6>{}).size());
}


