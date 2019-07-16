// Catch2
#include <catch2/catch.hpp>

// This repository
#include "button_filter.hpp"

TEST_CASE("ButtonFilterTest -- filterAlwaysHigh", "filterAlwaysHigh") {
	dr::ButtonFilter bf(true, false);

	// pass through high signals always
	REQUIRE(bf.filter(false) == false);
	REQUIRE(bf.filter(true)  == true);
	REQUIRE(bf.filter(true)  == true);
}

TEST_CASE("ButtonFilterTest -- filterAlwaysLow", "filterAlwaysLow") {
	dr::ButtonFilter bf(false, true);

	// pass through low signals always
	REQUIRE(bf.filter(false) == true);
	REQUIRE(bf.filter(true)  == true);
	REQUIRE(bf.filter(true)  == false);
}

TEST_CASE("ButtonFilterTest -- filterAlwaysHighAndLow", "filterAlwaysHighAndLow") {
	dr::ButtonFilter bf(true, true);

	// pass through any signal
	REQUIRE(bf.filter(false) == true);
	REQUIRE(bf.filter(false) == true);
	REQUIRE(bf.filter(true)  == true);
	REQUIRE(bf.filter(true)  == true);
}

TEST_CASE("ButtonFilterTest -- filterRisingEdge", "filterRisingEdge") {
	dr::ButtonFilter bf;

	// test rising edge
	REQUIRE(bf.filter(false) == false);
	REQUIRE(bf.filter(true)  == true);
}

TEST_CASE("ButtonFilterTest -- filterFallingEdge", "filterFallingEdge") {
	dr::ButtonFilter bf;

	// test falling edge
	REQUIRE(bf.filter(true)  == true);
	REQUIRE(bf.filter(false) == true);
}

TEST_CASE("ButtonFilterTest -- filterMultiple", "filterMultiple") {
	dr::ButtonFilter bf;

	// test multiple signals
	REQUIRE(bf.filter(true)  == true);
	REQUIRE(bf.filter(true)  == false);

	REQUIRE(bf.filter(false) == true);
	REQUIRE(bf.filter(false) == false);

	REQUIRE(bf.filter(true)  == true);
	REQUIRE(bf.filter(true)  == false);
}

/*
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
*/
