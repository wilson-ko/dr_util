// C++
#include <cmath>

// Catch2
#include <catch2/catch.hpp>

// This repository
#include "polynomial.hpp"

namespace {
	/// Test if two float numbers are equal.
	void testEqual(float const & a, float const & b)
	{
		REQUIRE_THAT(a, Catch::WithinULP(b, 0));
	}
}

TEST_CASE("PolynomialTest -- emptyPolynomial", "emptyPolynomial") {
	dr::Polynomial p;
	testEqual(0, p.y(0.0));
	testEqual(0, p.y(0.5));
	testEqual(0, p.y(1.0));
	testEqual(0, p.y(-0.5));
	testEqual(0, p.y(-1.0));
}

TEST_CASE("PolynomialTest -- constantPolynomial", "constantPolynomial") {
	dr::Polynomial p;
	p.terms.emplace_back(3.25, 0);

	testEqual(3.25, p.y( 0.0));
	testEqual(3.25, p.y( 0.5));
	testEqual(3.25, p.y( 1.0));
	testEqual(3.25, p.y(-0.5));
	testEqual(3.25, p.y(-1.0));
}

TEST_CASE("PolynomialTest -- squareRoot", "squareRoot") {
	dr::Polynomial p;
	p.terms.emplace_back(1, 0.5);

	testEqual( 0.000, p.y(0.0));
	testEqual( 1.000, p.y(1.0));
	testEqual( 2.000, p.y(4.0));

	REQUIRE(std::isnan(p.y(-1.0)) == true);
	REQUIRE(std::isnan(p.y(-4.0)) == true);
}

TEST_CASE("PolynomialTest -- firstOrder", "firstOrder") {
	dr::Polynomial p;
	p.terms.emplace_back(2, 1);
	testEqual( 0, p.y(0.0));
	testEqual( 1, p.y(0.5));
	testEqual( 2, p.y(1.0));
	testEqual(-1, p.y(-0.5));
	testEqual(-2, p.y(-1.0));
}

TEST_CASE("PolynomialTest -- thirdOrder", "thirdOrder") {
	dr::Polynomial p;
	p.terms.emplace_back(2,   1);
	p.terms.emplace_back(1.5, 3);

	testEqual( 0.0000, p.y(0.0));
	testEqual( 1.1875, p.y(0.5));
	testEqual( 3.5000, p.y(1.0));
	testEqual(-1.1875, p.y(-0.5));
	testEqual(-3.5000, p.y(-1.0));
}

TEST_CASE("PolynomialTest -- fourthOrder", "fourthOrder") {
	dr::Polynomial p;
	p.terms.emplace_back(1, 2);
	p.terms.emplace_back(2, 4);

	testEqual( 0.000, p.y(0.0));
	testEqual( 0.375, p.y(0.5));
	testEqual( 3.000, p.y(1.0));
	testEqual(36.000, p.y(2.0));
	testEqual( 0.375, p.y(-0.5));
	testEqual( 3.000, p.y(-1.0));
	testEqual(36.000, p.y(-2.0));
}

/*
#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(PolynomialTest, emptyPolynomial) {
	Polynomial p;
	ASSERT_DOUBLE_EQ(0, p.y(0.0));
	ASSERT_DOUBLE_EQ(0, p.y(0.5));
	ASSERT_DOUBLE_EQ(0, p.y(1.0));
	ASSERT_DOUBLE_EQ(0, p.y(-0.5));
	ASSERT_DOUBLE_EQ(0, p.y(-1.0));
}

TEST(PolynomialTest, constantPolynomial) {
	Polynomial p;
	p.terms.emplace_back(3.25, 0);

	ASSERT_DOUBLE_EQ(3.25, p.y( 0.0));
	ASSERT_DOUBLE_EQ(3.25, p.y( 0.5));
	ASSERT_DOUBLE_EQ(3.25, p.y( 1.0));
	ASSERT_DOUBLE_EQ(3.25, p.y(-0.5));
	ASSERT_DOUBLE_EQ(3.25, p.y(-1.0));
}

TEST(PolynomialTest, squareRoot) {
	Polynomial p;
	p.terms.emplace_back(1, 0.5);

	ASSERT_DOUBLE_EQ( 0.000, p.y(0.0));
	ASSERT_DOUBLE_EQ( 1.000, p.y(1.0));
	ASSERT_DOUBLE_EQ( 2.000, p.y(4.0));

	ASSERT_TRUE(std::isnan(p.y(-1.0)));
	ASSERT_TRUE(std::isnan(p.y(-4.0)));
}

TEST(PolynomialTest, firstOrder) {
	Polynomial p;
	p.terms.emplace_back(2, 1);
	ASSERT_DOUBLE_EQ( 0, p.y(0.0));
	ASSERT_DOUBLE_EQ( 1, p.y(0.5));
	ASSERT_DOUBLE_EQ( 2, p.y(1.0));
	ASSERT_DOUBLE_EQ(-1, p.y(-0.5));
	ASSERT_DOUBLE_EQ(-2, p.y(-1.0));
}

TEST(PolynomialTest, thirdOrder) {
	Polynomial p;
	p.terms.emplace_back(2,   1);
	p.terms.emplace_back(1.5, 3);

	ASSERT_DOUBLE_EQ( 0.0000, p.y(0.0));
	ASSERT_DOUBLE_EQ( 1.1875, p.y(0.5));
	ASSERT_DOUBLE_EQ( 3.5000, p.y(1.0));
	ASSERT_DOUBLE_EQ(-1.1875, p.y(-0.5));
	ASSERT_DOUBLE_EQ(-3.5000, p.y(-1.0));
}

TEST(PolynomialTest, fourthOrder) {
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

}
*/
