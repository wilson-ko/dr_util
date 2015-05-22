#include "button_filter.hpp"

#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(ButtonFilterTest, filterAlwaysHigh) {
	ButtonFilter bf(true, false);

	// pass through high signals always
	EXPECT_FALSE(bf.filter(false));
	EXPECT_TRUE(bf.filter(true));
	EXPECT_TRUE(bf.filter(true));
}

TEST(ButtonFilterTest, filterAlwaysLow) {
	ButtonFilter bf(false, true);

	// pass through low signals always
	EXPECT_TRUE(bf.filter(false));
	EXPECT_TRUE(bf.filter(true));
	EXPECT_FALSE(bf.filter(true));
}

TEST(ButtonFilterTest, filterAlwaysHighAndLow) {
	ButtonFilter bf(true, true);

	// pass through any signal
	EXPECT_TRUE(bf.filter(false));
	EXPECT_TRUE(bf.filter(false));
	EXPECT_TRUE(bf.filter(true));
	EXPECT_TRUE(bf.filter(true));
}

TEST(ButtonFilterTest, filterRisingEdge) {
	ButtonFilter bf;

	// test rising edge
	EXPECT_FALSE(bf.filter(false));
	EXPECT_TRUE(bf.filter(true));
}

TEST(ButtonFilterTest, filterFallingEdge) {
	ButtonFilter bf;

	// test falling edge
	EXPECT_TRUE(bf.filter(true));
	EXPECT_TRUE(bf.filter(false));
}

TEST(ButtonFilterTest, filterMultiple) {
	ButtonFilter bf;

	// test multiple signals
	EXPECT_TRUE(bf.filter(true));
	EXPECT_FALSE(bf.filter(true));

	EXPECT_TRUE(bf.filter(false));
	EXPECT_FALSE(bf.filter(false));

	EXPECT_TRUE(bf.filter(true));
	EXPECT_FALSE(bf.filter(true));
}

}
