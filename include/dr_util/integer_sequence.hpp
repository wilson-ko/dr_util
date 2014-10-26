#pragma once
#include <cstddef>

namespace dr {

	/// Type to represent an integer sequence.
	template<typename T, T... Ints> struct integer_sequence {
		using value_type = T;
		constexpr static std::size_t size() noexcept { return sizeof...(Ints); }
	};

	/// Short-hand for size_t integer sequence.
	template<std::size_t... Ints>
	using index_sequence = integer_sequence<std::size_t, Ints...>;

	namespace impl {
		/// Make an integer sequence from start to end with increments of 1.
		template<typename T, T start, T end, bool, T... I> struct make_integer_slice;

		template<typename T, T start, T end, T... I> struct make_integer_slice<T, start, end, false, I...> {
			using type = typename make_integer_slice<T, start, end - 1, start == end - 1, end - 1, I...>::type;
		};

		template<typename T, T start, T end, T... I> struct make_integer_slice<T, start, end, true, I...> {
			using type = integer_sequence<T, I...>;
		};
	}

	/// Make an integer sequence [0, N).
	template<typename T, T N>
	using make_integer_sequence = typename impl::make_integer_slice<T, 0, N, 0 == N>::type;

	/// Make an integer sequence [start, end).
	template<typename T, T start, T end>
	using make_integer_slice = typename impl::make_integer_slice<T, start, end, start == end>::type;

	/// Make an index sequence [0, N).
	template<std::size_t N>
	using make_index_sequence = make_integer_sequence<std::size_t, N>;

	/// Make an index sequence [start, end).
	template<std::size_t start, std::size_t end>
	using make_index_slice = make_integer_slice<std::size_t, start, end>;

	/// Make an index sequence [0, sizeof...(Args)).
	template<typename... Args>
	using index_sequence_for = make_index_sequence<sizeof...(Args)>;
}
