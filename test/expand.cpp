#include "expand.hpp"

#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(Expand, simple) {
	EXPECT_EQ(expandVariables("$test", {{"test", "foo"}}), "foo");
	EXPECT_EQ(expandVariables("$test", {{"test", "bar"}}), "bar");
	EXPECT_EQ(expandVariables("${test}", {{"test", "foo"}}), "foo");
	EXPECT_EQ(expandVariables("${test}", {{"test", "bar"}}), "bar");
}

TEST(Expand, sentence) {
	EXPECT_EQ(expandVariables("Hello $name, welcome to $place", {{"name", "Rick"}, {"place", "Earth"}}), "Hello Rick, welcome to Earth");
	EXPECT_EQ(expandVariables("Hello ${name}, welcome to ${place}", {{"name", "Rick"}, {"place", "Earth"}}), "Hello Rick, welcome to Earth");
}

TEST(Expand, nipple_brackets) {
	EXPECT_EQ(expandVariables("test$testtest", {{"test", "wrong"}, {"testtest", "good"}}), "testgood");
	EXPECT_EQ(expandVariables("test${test}test", {{"test", "good"}, {"testtest", "wrong"}}), "testgoodtest");
}

TEST(Expand, ignore_empty_key) {
	EXPECT_EQ(expandVariables("$", {{"", "aap"}}), "$");
	EXPECT_EQ(expandVariables("${}", {{"", "aap"}}), "${}");
}

}
