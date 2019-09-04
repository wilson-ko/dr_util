#include "polynomial.hpp"

#include <gtest/gtest.h>

#include <cmath>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(PolynomialTest, emptyPolynomial) { //NOLINT
	Polynomial p;
	ASSERT_DOUBLE_EQ(0, p.y(0.0));
	ASSERT_DOUBLE_EQ(0, p.y(0.5));
	ASSERT_DOUBLE_EQ(0, p.y(1.0));
	ASSERT_DOUBLE_EQ(0, p.y(-0.5));
	ASSERT_DOUBLE_EQ(0, p.y(-1.0));
}

TEST(PolynomialTest, constantPolynomial) { //NOLINT
	Polynomial p;
	p.terms.emplace_back(3.25, 0); //NOLINT

	ASSERT_DOUBLE_EQ(3.25, p.y( 0.0));
	ASSERT_DOUBLE_EQ(3.25, p.y( 0.5));
	ASSERT_DOUBLE_EQ(3.25, p.y( 1.0));
	ASSERT_DOUBLE_EQ(3.25, p.y(-0.5));
	ASSERT_DOUBLE_EQ(3.25, p.y(-1.0));
}

TEST(PolynomialTest, squareRoot) { //NOLINT
	Polynomial p;
	p.terms.emplace_back(1, 0.5);

	ASSERT_DOUBLE_EQ( 0.000, p.y(0.0));
	ASSERT_DOUBLE_EQ( 1.000, p.y(1.0));
	ASSERT_DOUBLE_EQ( 2.000, p.y(4.0));

	ASSERT_TRUE(std::isnan(p.y(-1.0)));
	ASSERT_TRUE(std::isnan(p.y(-4.0)));
}

TEST(PolynomialTest, firstOrder) { //NOLINT
	Polynomial p;
	p.terms.emplace_back(2, 1);
	ASSERT_DOUBLE_EQ( 0, p.y(0.0));
	ASSERT_DOUBLE_EQ( 1, p.y(0.5));
	ASSERT_DOUBLE_EQ( 2, p.y(1.0));
	ASSERT_DOUBLE_EQ(-1, p.y(-0.5));
	ASSERT_DOUBLE_EQ(-2, p.y(-1.0));
}

TEST(PolynomialTest, thirdOrder) { //NOLINT
	Polynomial p;
	p.terms.emplace_back(2,   1);
	p.terms.emplace_back(1.5, 3); //NOLINT

	ASSERT_DOUBLE_EQ( 0.0000, p.y(0.0));
	ASSERT_DOUBLE_EQ( 1.1875, p.y(0.5));
	ASSERT_DOUBLE_EQ( 3.5000, p.y(1.0));
	ASSERT_DOUBLE_EQ(-1.1875, p.y(-0.5));
	ASSERT_DOUBLE_EQ(-3.5000, p.y(-1.0));
}

TEST(PolynomialTest, fourthOrder) { //NOLINT
	Polynomial p;
	p.terms.emplace_back(1, 2);
	p.terms.emplace_back(2, 4);

	ASSERT_DOUBLE_EQ( 0.000, p.y(0.0));
	ASSERT_DOUBLE_EQ( 0.375, p.y(0.5));
	ASSERT_DOUBLE_EQ( 3.000, p.y(1.0));
	ASSERT_DOUBLE_EQ(36.000, p.y(2.0));
	ASSERT_DOUBLE_EQ( 0.375, p.y(-0.5));
	ASSERT_DOUBLE_EQ( 3.000, p.y(-1.0));
	ASSERT_DOUBLE_EQ(36.000, p.y(-2.0));
}

} //namespace dr
