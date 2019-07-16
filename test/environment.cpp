// C
#include <stdlib.h>

// Catch2
#include <catch2/catch.hpp>

// This repository
#include "environment.hpp"

TEST_CASE("Environment -- getEnvironment", "getEnvironment") {
	::clearenv();
	REQUIRE(dr::getEnvironment() == (std::map<std::string, std::string>{}));
	::setenv("FOO", "aap", true);
	REQUIRE(dr::getEnvironment() == (std::map<std::string, std::string>{{"FOO", "aap"}}));
	::setenv("BAR", "noot", true);
	REQUIRE(dr::getEnvironment() == (std::map<std::string, std::string>{{"FOO", "aap"}, {"BAR", "noot"}}));
}

/*
#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(Environment, getEnvironment) {
	::clearenv();
	EXPECT_EQ(getEnvironment(), (std::map<std::string, std::string>{}));
	::setenv("FOO", "aap", true);
	EXPECT_EQ(getEnvironment(), (std::map<std::string, std::string>{{"FOO", "aap"}}));
	::setenv("BAR", "noot", true);
	EXPECT_EQ(getEnvironment(), (std::map<std::string, std::string>{{"FOO", "aap"}, {"BAR", "noot"}}));
}

}
*/
