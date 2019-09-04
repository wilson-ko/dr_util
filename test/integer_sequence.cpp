#include <string>
#include <sstream>
#include <array>

#include <gtest/gtest.h>

#include "integer_sequence.hpp"

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

template<typename T1, typename T2>
constexpr bool have_same_types(T1 const & /* type1 */, T2 const & /* type2 */) {
	return std::is_same<T1, T2>{};
}

TEST(IntegerSequence, have_same_types) { //NOLINT
	bool b1;
	bool b2;
	bool const b3 = false;
	bool volatile b4;
	bool * bp;
	int i;

	static_assert(have_same_types(b1, b2));
	static_assert(have_same_types(b1, b3));
	static_assert(not have_same_types(b1, b4));
	static_assert(not have_same_types(b1, bp));
	static_assert(not have_same_types(b1, i));
}

TEST(IntegerSequence, add_offset_int) { //NOLINT
	auto offset_sequence   = offset_integer_sequence<int, 5>(std::make_integer_sequence<int, 3>{}); //NOLINT
	auto expected_sequence = std::integer_sequence<int, 5, 6, 7>{}; //NOLINT
	static_assert(have_same_types(offset_sequence, expected_sequence));
	ASSERT_TRUE(true);
}

TEST(IntegerSequence, add_offset_index) { //NOLINT
	auto offset_sequence   = offset_index_sequence<8>(std::make_index_sequence<3>{}); //NOLINT
	auto expected_sequence = std::index_sequence<8, 9, 10>{}; //NOLINT
	static_assert(have_same_types(offset_sequence, expected_sequence));
	ASSERT_TRUE(true);
}

} //namespace dr
