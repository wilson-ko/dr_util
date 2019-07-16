// Catch2
#include <catch2/catch.hpp>

// This repository
#include "bitwise.hpp"

TEST_CASE("BitwiseTest -- makeBitMask", "makeBitMask") {
	// least significant bit
	REQUIRE(0x1        == dr::bitMask(0));
	// some middle bit
	REQUIRE(0x4000     == dr::bitMask(14));
	// most significant bit
	REQUIRE(0x80000000 == dr::bitMask(31));
}

TEST_CASE("BitwiseTest -- testTestBit", "testTestBit") {
	// least significant bit
	REQUIRE(dr::testBit(0x1, 0)  == true);
	REQUIRE(dr::testBit(0x1, 14) == false);
	REQUIRE(dr::testBit(0x1, 31) == false);

	// some middle bit
	REQUIRE(dr::testBit(0x4000, 0)  == false);
	REQUIRE(dr::testBit(0x4000, 14) == true);
	REQUIRE(dr::testBit(0x4000, 31) == false);

	// most significant bit
	REQUIRE(dr::testBit(0x80000000, 0)  == false);
	REQUIRE(dr::testBit(0x80000000, 14) == false);
	REQUIRE(dr::testBit(0x80000000, 31) == true);

	// no value
	REQUIRE(dr::testBit(0, 0) == false);
	REQUIRE(dr::testBit(0, 14) == false);
	REQUIRE(dr::testBit(0, 31) == false);

	// all bits set to true
	REQUIRE(dr::testBit(0xFFFFFFFF, 0)  == true);
	REQUIRE(dr::testBit(0xFFFFFFFF, 14) == true);
	REQUIRE(dr::testBit(0xFFFFFFFF, 31) == true);
}

TEST_CASE("BitwiseTest -- testTestRising", "testTestRising") {
	// no change for all bits set to 1
	REQUIRE(dr::testRising(0xFFFFFFFF, 0xFFFFFFFF, 0) == false);
	REQUIRE(dr::testRising(0xFFFFFFFF, 0xFFFFFFFF, 14) == false);
	REQUIRE(dr::testRising(0xFFFFFFFF, 0xFFFFFFFF, 31) == false);

	// no change for all bits set to 0
	REQUIRE(dr::testRising(0, 0, 0)  == false);
	REQUIRE(dr::testRising(0, 0, 14) == false);
	REQUIRE(dr::testRising(0, 0, 31) == false);

	// ignore falling value
	REQUIRE(dr::testRising(0xFFFFFFFF, 0, 0)  == true);
	REQUIRE(dr::testRising(0xFFFFFFFF, 0, 14) == true);
	REQUIRE(dr::testRising(0xFFFFFFFF, 0, 31) == true);

	// test rising edge
	REQUIRE(dr::testRising(0, 0xFFFFFFFF, 0)  == false);
	REQUIRE(dr::testRising(0, 0xFFFFFFFF, 14) == false);
	REQUIRE(dr::testRising(0, 0xFFFFFFFF, 31) == false);

	// test specific rising edge
	REQUIRE(dr::testRising(0x1, 0, 0)         == true);
	REQUIRE(dr::testRising(0x4000, 0, 14)     == true);
	REQUIRE(dr::testRising(0x80000000, 0, 31) == true);

	// test wrong rising edge
	REQUIRE(dr::testRising(0x2, 0, 0)         == false);
	REQUIRE(dr::testRising(0x8000, 0, 14)     == false);
	REQUIRE(dr::testRising(0x40000000, 0, 31) == false);
}

/*
#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(BitwiseTest, makeBitMask) {
	// least significant bit
	EXPECT_EQ(0x1,        bitMask(0));
	// some middle bit
	EXPECT_EQ(0x4000,     bitMask(14));
	// most significant bit
	EXPECT_EQ(0x80000000, bitMask(31));
}

TEST(BitwiseTest, testTestBit) {
	// least significant bit
	EXPECT_TRUE(testBit(0x1, 0));
	EXPECT_FALSE(testBit(0x1, 14));
	EXPECT_FALSE(testBit(0x1, 31));

	// some middle bit
	EXPECT_FALSE(testBit(0x4000, 0));
	EXPECT_TRUE(testBit(0x4000, 14));
	EXPECT_FALSE(testBit(0x4000, 31));

	// most significant bit
	EXPECT_FALSE(testBit(0x80000000, 0));
	EXPECT_FALSE(testBit(0x80000000, 14));
	EXPECT_TRUE(testBit(0x80000000, 31));

	// no value
	EXPECT_FALSE(testBit(0, 0));
	EXPECT_FALSE(testBit(0, 14));
	EXPECT_FALSE(testBit(0, 31));

	// all bits set to true
	EXPECT_TRUE(testBit(0xFFFFFFFF, 0));
	EXPECT_TRUE(testBit(0xFFFFFFFF, 14));
	EXPECT_TRUE(testBit(0xFFFFFFFF, 31));
}

TEST(BitwiseTest, testTestRising) {
	// no change for all bits set to 1
	EXPECT_FALSE(testRising(0xFFFFFFFF, 0xFFFFFFFF, 0));
	EXPECT_FALSE(testRising(0xFFFFFFFF, 0xFFFFFFFF, 14));
	EXPECT_FALSE(testRising(0xFFFFFFFF, 0xFFFFFFFF, 31));

	// no change for all bits set to 0
	EXPECT_FALSE(testRising(0, 0, 0));
	EXPECT_FALSE(testRising(0, 0, 14));
	EXPECT_FALSE(testRising(0, 0, 31));

	// ignore falling value
	EXPECT_TRUE(testRising(0xFFFFFFFF, 0, 0));
	EXPECT_TRUE(testRising(0xFFFFFFFF, 0, 14));
	EXPECT_TRUE(testRising(0xFFFFFFFF, 0, 31));

	// test rising edge
	EXPECT_FALSE(testRising(0, 0xFFFFFFFF, 0));
	EXPECT_FALSE(testRising(0, 0xFFFFFFFF, 14));
	EXPECT_FALSE(testRising(0, 0xFFFFFFFF, 31));

	// test specific rising edge
	EXPECT_TRUE(testRising(0x1, 0, 0));
	EXPECT_TRUE(testRising(0x4000, 0, 14));
	EXPECT_TRUE(testRising(0x80000000, 0, 31));

	// test wrong rising edge
	EXPECT_FALSE(testRising(0x2, 0, 0));
	EXPECT_FALSE(testRising(0x8000, 0, 14));
	EXPECT_FALSE(testRising(0x40000000, 0, 31));
}

}
*/
