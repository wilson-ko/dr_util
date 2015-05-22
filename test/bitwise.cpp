#include "bitwise.hpp"

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
