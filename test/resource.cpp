// Catch2
#include <catch2/catch.hpp>

// ROS

// This repository
// #include "resource.hpp"

// #include <ros/package.h>

TEST_CASE("ResourceTest -- packageUrl", "packageUrl") {
	REQUIRE(ros::package::getPath("dr_util") + "/test.file" == rosUrlToPath("package://dr_util/test.file"));
}

TEST_CASE("ResourceTest -- localFileUrl", "localFileUrl") {
	REQUIRE("/test.file" == rosUrlToPath("file:///test.file"));
}

TEST_CASE("ResourceTest - remoteFileUrl", "remoteFileUrl") {
	REQUIRE_THROWS(rosUrlToPath("file://host/test.file"));
}

TEST_CASE("ResourceTest -- unsupportedScheme", "unsupportedScheme") {
	REQUIRE_THROWS(rosUrlToPath("http://example.com/test.file"));
}

/*
#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(ResourceTest, packageUrl) {
	ASSERT_EQ(ros::package::getPath("dr_util") + "/test.file", rosUrlToPath("package://dr_util/test.file"));
}

TEST(ResourceTest, localFileUrl) {
	ASSERT_EQ("/test.file", rosUrlToPath("file:///test.file"));
}

TEST(ResourceTest, remoteFileUrl) {
	ASSERT_ANY_THROW(rosUrlToPath("file://host/test.file"));
}

TEST(ResourceTest, unsupportedScheme) {
	ASSERT_ANY_THROW(rosUrlToPath("http://example.com/test.file"));
}

}
*/
