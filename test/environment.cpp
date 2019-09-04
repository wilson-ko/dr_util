#include "environment.hpp"

#include <cstdlib>
#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(Environment, getEnvironment) { //NOLINT
	::clearenv();
	EXPECT_EQ(getEnvironment(), (std::map<std::string, std::string>{}));
	::setenv("FOO", "aap", 1);
	EXPECT_EQ(getEnvironment(), (std::map<std::string, std::string>{{"FOO", "aap"}}));
	::setenv("BAR", "noot", 1);
	EXPECT_EQ(getEnvironment(), (std::map<std::string, std::string>{{"FOO", "aap"}, {"BAR", "noot"}}));
}

} //namespace dr
