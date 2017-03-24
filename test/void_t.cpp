#include "void_t.hpp"

#include <gtest/gtest.h>

using namespace dr;

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(VoidT, type) {
	static_assert(std::is_same<void, void_t<>>{}, "");
	static_assert(std::is_same<void, void_t<int>>{}, "");
	static_assert(std::is_same<void, void_t<bool>>{}, "");
	static_assert(std::is_same<void, void_t<bool, int>>{}, "");
	ASSERT_TRUE(true);
}

}
