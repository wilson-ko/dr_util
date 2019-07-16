/// C++
#include <string>
#include <optional>

// Catch2
#include <catch2/catch.hpp>

// This repository
#include "geometry.hpp"

namespace {
	/// Test if two float numbers are within tolerance of each other.
	void testNear(float const & a, float const & b, float const & tolerance) {
		auto diff = std::abs(a - b);
		if (diff <= tolerance) {
			SUCCEED();
		} else {
			FAIL(
				"first argument " + std::to_string(a) +
				"differs from the second argument " + std::to_string(b) +
				"in " + std::to_string(diff) +
				"which is larger than the allowed tolerance " + std::to_string(tolerance));
		}
	}

	void testEqual(float const & a, float const & b)
	{
		REQUIRE_THAT(a, Catch::WithinULP(b, 0));
	}
}

TEST_CASE("GeometryTest -- testPi", "testPi") {
	// test equality with C defined pi
	testEqual(dr::pi(), M_PI);
}

TEST_CASE("GeometryTest -- testDegrees", "testDegrees") {
	// test normal use case
	testEqual(dr::degrees(0), 0);
	testEqual(dr::degrees(180), M_PI);
	testEqual(dr::degrees(360), 2 * M_PI);
	testNear(dr::degrees(37), 0.645771823237901, 0.0000001); // 37 / 180 * pi

	// test negativity
	testEqual(dr::degrees(-180), -M_PI);
	testEqual(dr::degrees(-360), -2 * M_PI);
}

TEST_CASE("GeometryTest -- testToDegrees", "testToDegrees") {
	// test normal use case
	testEqual(dr::to_degrees(0), 0);
	testEqual(dr::to_degrees(M_PI), 180);
	testEqual(dr::to_degrees(2 * M_PI), 360);
	testNear(dr::to_degrees(0.645771823237901), 37, 0.0000001); // 37 / 180 * pi

	// test negativity
	testEqual(dr::to_degrees(-M_PI), -180);
	testEqual(dr::to_degrees(-2 * M_PI), -360);
}

TEST_CASE("GeometryTest -- testSign", "testSign") {
	// normal use cases
	REQUIRE(dr::sign(-1) == -1);
	REQUIRE(dr::sign(1)  ==  1);
	REQUIRE(dr::sign(0)  ==  0);

	// double use cases
	testEqual(dr::sign(M_PI), 1);
	testEqual(dr::sign(-M_PI), -1);
	testEqual(dr::sign(std::numeric_limits<double>::quiet_NaN()), 0);
}

TEST_CASE("GeometryTest -- testNormalizeAngle", "testNormalizeAngle") {
	// normal use
	testEqual(dr::normalizeAngle(3 * M_PI), M_PI);
	testEqual(dr::normalizeAngle(3.5 * M_PI), 1.5 * M_PI);

	// negativity
	testEqual(dr::normalizeAngle(-3 * M_PI), M_PI);
	testEqual(dr::normalizeAngle(-3.5 * M_PI), 0.5 * M_PI);
}

TEST_CASE("GeometryTest -- testAngleFrom", "testAngleFrom") {
	// same as normalizeAngle

	// normal use
	testEqual(dr::angleFrom(3 * M_PI,   0), M_PI);
	testEqual(dr::angleFrom(3.5 * M_PI, 0), 1.5 * M_PI);

	// negative angle
	testEqual(dr::angleFrom(-3 * M_PI,   0), M_PI);
	testEqual(dr::angleFrom(-3.5 * M_PI, 0), 0.5 * M_PI);

	// set different angle

	// normal use
	testEqual(dr::angleFrom(3 * M_PI,   5 * M_PI), 5 * M_PI);
	testEqual(dr::angleFrom(3.5 * M_PI, 5 * M_PI), 5.5 * M_PI);

	// negative from angle
	testEqual(dr::angleFrom(-3 * M_PI,   -5 * M_PI), -5 * M_PI);
	testEqual(dr::angleFrom(-3.5 * M_PI, -5 * M_PI), -3.5 * M_PI);

}

TEST_CASE("GeometryTest -- testSquare", "testSquare") {
	testEqual(dr::square(2), 4);
	testEqual(dr::square(-2), 4);
	testNear(dr::square(M_PI), 9.86960440108935861883, 0.000001);
	testNear(dr::square(-M_PI), 9.86960440108935861883, 0.000001);
}

TEST_CASE("GeometryTest -- testSquareSum", "testSquareSum") {
	testEqual(dr::square_sum(2.0, 2.0), 8.0);
	testEqual(dr::square_sum(1.0, 2.0, 3.0, 4.0, 5.0), 55.0);
	testEqual(dr::square_sum(-1.0, -2.0, -3.0, -4.0, -5.0), 55.0);
}

TEST_CASE("GeometryTest -- testLength", "testLength") {
	testEqual(dr::length(1.0, 0.0, 0.0), 1.0);
	testNear(dr::length(10.0, 10.0, 0.0), 14.1421356237309504880, 0.0000001); // sqrt(10*10 + 10*10)
	testEqual(dr::length(0.0, 0.0, 0.0), 0.0);
}

TEST_CASE("GeometryTest -- testAngle", "testAngle") {
	testEqual(dr::angle(std::sqrt(2), 1.0, 1.0), M_PI_2);
}


/*
#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(GeometryTest, testPi) {
	// test equality with C defined pi
	EXPECT_DOUBLE_EQ(pi(), M_PI);
}

TEST(GeometryTest, testDegrees) {
	// test normal use case
	EXPECT_DOUBLE_EQ(degrees(0), 0);
	EXPECT_DOUBLE_EQ(degrees(180), M_PI);
	EXPECT_DOUBLE_EQ(degrees(360), 2 * M_PI);
	EXPECT_NEAR(degrees(37), 0.645771823237901, 0.0000001); // 37 / 180 * pi

	// test negativity
	EXPECT_DOUBLE_EQ(degrees(-180), -M_PI);
	EXPECT_DOUBLE_EQ(degrees(-360), -2 * M_PI);
}

TEST(GeometryTest, testToDegrees) {
	// test normal use case
	EXPECT_DOUBLE_EQ(to_degrees(0), 0);
	EXPECT_DOUBLE_EQ(to_degrees(M_PI), 180);
	EXPECT_DOUBLE_EQ(to_degrees(2 * M_PI), 360);
	EXPECT_NEAR(to_degrees(0.645771823237901), 37, 0.0000001); // 37 / 180 * pi

	// test negativity
	EXPECT_DOUBLE_EQ(to_degrees(-M_PI), -180);
	EXPECT_DOUBLE_EQ(to_degrees(-2 * M_PI), -360);
}

TEST(GeometryTest, testSign) {
	// normal use cases
	EXPECT_EQ(sign(-1), -1);
	EXPECT_EQ(sign(1), 1);
	EXPECT_EQ(sign(0), 0);

	// double use cases
	EXPECT_DOUBLE_EQ(sign(M_PI), 1);
	EXPECT_DOUBLE_EQ(sign(-M_PI), -1);
	EXPECT_DOUBLE_EQ(sign(std::numeric_limits<double>::quiet_NaN()), 0);
}

TEST(GeometryTest, testNormalizeAngle) {
	// normal use
	EXPECT_DOUBLE_EQ(normalizeAngle(3 * M_PI), M_PI);
	EXPECT_DOUBLE_EQ(normalizeAngle(3.5 * M_PI), 1.5 * M_PI);

	// negativity
	EXPECT_DOUBLE_EQ(normalizeAngle(-3 * M_PI), M_PI);
	EXPECT_DOUBLE_EQ(normalizeAngle(-3.5 * M_PI), 0.5 * M_PI);
}

TEST(GeometryTest, testAngleFrom) {
	// same as normalizeAngle

	// normal use
	EXPECT_DOUBLE_EQ(angleFrom(3 * M_PI,   0), M_PI);
	EXPECT_DOUBLE_EQ(angleFrom(3.5 * M_PI, 0), 1.5 * M_PI);

	// negative angle
	EXPECT_DOUBLE_EQ(angleFrom(-3 * M_PI,   0), M_PI);
	EXPECT_DOUBLE_EQ(angleFrom(-3.5 * M_PI, 0), 0.5 * M_PI);

	// set different angle

	// normal use
	EXPECT_DOUBLE_EQ(angleFrom(3 * M_PI,   5 * M_PI), 5 * M_PI);
	EXPECT_DOUBLE_EQ(angleFrom(3.5 * M_PI, 5 * M_PI), 5.5 * M_PI);

	// negative from angle
	EXPECT_DOUBLE_EQ(angleFrom(-3 * M_PI,   -5 * M_PI), -5 * M_PI);
	EXPECT_DOUBLE_EQ(angleFrom(-3.5 * M_PI, -5 * M_PI), -3.5 * M_PI);

}

TEST(GeometryTest, testSquare) {
	EXPECT_DOUBLE_EQ(square(2), 4);
	EXPECT_DOUBLE_EQ(square(-2), 4);
	EXPECT_NEAR(square(M_PI), 9.86960440108935861883, 0.000001);
	EXPECT_NEAR(square(-M_PI), 9.86960440108935861883, 0.000001);
}

TEST(GeometryTest, testSquareSum) {
	EXPECT_DOUBLE_EQ(square_sum(2.0, 2.0), 8.0);
	EXPECT_DOUBLE_EQ(square_sum(1.0, 2.0, 3.0, 4.0, 5.0), 55.0);
	EXPECT_DOUBLE_EQ(square_sum(-1.0, -2.0, -3.0, -4.0, -5.0), 55.0);
}

TEST(GeometryTest, testLength) {
	EXPECT_DOUBLE_EQ(length(1.0, 0.0, 0.0), 1.0);
	EXPECT_NEAR(length(10.0, 10.0, 0.0), 14.1421356237309504880, 0.0000001); // sqrt(10*10 + 10*10)
	EXPECT_DOUBLE_EQ(length(0.0, 0.0, 0.0), 0.0);
}

TEST(GeometryTest, testAngle) {
	EXPECT_DOUBLE_EQ(angle(std::sqrt(2), 1.0, 1.0), M_PI_2);
}

}
*/
