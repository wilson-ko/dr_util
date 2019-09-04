#include "geometry.hpp"

#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(GeometryTest, testPi) { //NOLINT
	// test equality with C defined pi
	EXPECT_DOUBLE_EQ(pi(), M_PI);
}

TEST(GeometryTest, testDegrees) { //NOLINT
	// test normal use case
	EXPECT_DOUBLE_EQ(degrees(0), 0);
	EXPECT_DOUBLE_EQ(degrees(180), M_PI);
	EXPECT_DOUBLE_EQ(degrees(360), 2 * M_PI);
	EXPECT_NEAR(degrees(37), 0.645771823237901, 0.0000001); // 37 / 180 * pi

	// test negativity
	EXPECT_DOUBLE_EQ(degrees(-180), -M_PI);
	EXPECT_DOUBLE_EQ(degrees(-360), -2 * M_PI);
}

TEST(GeometryTest, testToDegrees) { //NOLINT
	// test normal use case
	EXPECT_DOUBLE_EQ(to_degrees(0), 0);
	EXPECT_DOUBLE_EQ(to_degrees(M_PI), 180);
	EXPECT_DOUBLE_EQ(to_degrees(2 * M_PI), 360);
	EXPECT_NEAR(to_degrees(0.645771823237901), 37, 0.0000001); // 37 / 180 * pi

	// test negativity
	EXPECT_DOUBLE_EQ(to_degrees(-M_PI), -180);
	EXPECT_DOUBLE_EQ(to_degrees(-2 * M_PI), -360);
}

TEST(GeometryTest, testSign) { //NOLINT
	// normal use cases
	EXPECT_EQ(sign(-1), -1);
	EXPECT_EQ(sign(1), 1);
	EXPECT_EQ(sign(0), 0);

	// double use cases
	EXPECT_DOUBLE_EQ(sign(M_PI), 1);
	EXPECT_DOUBLE_EQ(sign(-M_PI), -1);
	EXPECT_DOUBLE_EQ(sign(std::numeric_limits<double>::quiet_NaN()), 0);
}

TEST(GeometryTest, testNormalizeAngle) { //NOLINT
	// normal use
	EXPECT_DOUBLE_EQ(normalizeAngle(3 * M_PI), M_PI);
	EXPECT_DOUBLE_EQ(normalizeAngle(3.5 * M_PI), 1.5 * M_PI);

	// negativity
	EXPECT_DOUBLE_EQ(normalizeAngle(-3 * M_PI), M_PI);
	EXPECT_DOUBLE_EQ(normalizeAngle(-3.5 * M_PI), 0.5 * M_PI);
}

TEST(GeometryTest, testAngleFrom) { //NOLINT
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

TEST(GeometryTest, testSquare) { //NOLINT
	EXPECT_DOUBLE_EQ(square(2), 4);
	EXPECT_DOUBLE_EQ(square(-2), 4);
	EXPECT_NEAR(square(M_PI), 9.86960440108935861883, 0.000001);
	EXPECT_NEAR(square(-M_PI), 9.86960440108935861883, 0.000001);
}

TEST(GeometryTest, testSquareSum) { //NOLINT
	EXPECT_DOUBLE_EQ(square_sum(2.0, 2.0), 8.0);
	EXPECT_DOUBLE_EQ(square_sum(1.0, 2.0, 3.0, 4.0, 5.0), 55.0);
	EXPECT_DOUBLE_EQ(square_sum(-1.0, -2.0, -3.0, -4.0, -5.0), 55.0);
}

TEST(GeometryTest, testLength) { //NOLINT
	EXPECT_DOUBLE_EQ(length(1.0, 0.0, 0.0), 1.0);
	EXPECT_NEAR(length(10.0, 10.0, 0.0), 14.1421356237309504880, 0.0000001); // sqrt(10*10 + 10*10)
	EXPECT_DOUBLE_EQ(length(0.0, 0.0, 0.0), 0.0);
}

TEST(GeometryTest, testAngle) { //NOLINT
	EXPECT_DOUBLE_EQ(angle(std::sqrt(2), 1.0, 1.0), M_PI_2);
}

} //namespace dr
