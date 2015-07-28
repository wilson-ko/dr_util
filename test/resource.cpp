#include "resource.hpp"

#include <ros/package.h>

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
