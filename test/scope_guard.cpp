#include "scope_guard.hpp"

#include <gtest/gtest.h>

#include <type_traits>

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

static_assert(std::is_copy_constructible_v <ScopeGuard<void(*)()>> == false);
static_assert(std::is_copy_assignable_v    <ScopeGuard<void(*)()>> == false);
static_assert(std::is_move_constructible_v <ScopeGuard<void(*)()>> == true );
static_assert(std::is_move_assignable_v    <ScopeGuard<void(*)()>> == true );

TEST(ScopeGuard, trigger) {
	int triggered = 0;

	{
		ASSERT_EQ(triggered, 0);
		auto guard = scopeGuard([&] { ++triggered; });
		ASSERT_EQ(triggered, 0);
	}

	ASSERT_EQ(triggered, 1);
}

TEST(ScopeGuard, reset) {
	int triggered = 0;

	{
		auto guard = scopeGuard([&] { ++triggered; });
		guard.reset();
	}

	ASSERT_EQ(triggered, 0);
}

TEST(ScopeGuard, move) {
	int triggered = 0;

	{
		auto guard = scopeGuard([&] { ++triggered; });
		auto other_guard = std::move(guard);
	}

	ASSERT_EQ(triggered, 1);
}

TEST(ScopeGuard, move_reset) {
	int triggered = 0;

	{
		auto guard = scopeGuard([&] { ++triggered; });
		auto other_guard = std::move(guard);
		other_guard.reset();
	}

	ASSERT_EQ(triggered, 0);
}

}
