#pragma once
#include <integer_sequence.hpp>

#include <cstddef>
#include <tuple>

namespace dr {

/// Make a tuple as a subset of a given tuple where an index sequence determines which elements to select.
template<typename... Args, std::size_t... I>
auto tuple_select(std::tuple<Args...> const & tuple, std::index_sequence<I...>) {
	return std::make_tuple(std::get<I>(tuple)...);
}

/// Get a slice from a tuple as a new tuple.
/**
 * \return A tuple containing a slice of the original tuple.
 */
template<std::size_t start, std::size_t count, typename... Args >
auto tuple_slice(std::tuple<Args...> const & tuple) {
	return tuple_select(tuple, offset_index_sequence<start>(std::make_index_sequence<count>{}));
}

/// Get the tail of a tuple.
/**
 * \return A tuple containing all but the head element of the original tuple.
 */
template<typename Head>
std::tuple<> tuple_tail(std::tuple<Head> const & tuple) {
	(void) tuple;
	return std::tuple<>{};
}

/// Get the tail of a tuple.
/**
 * \return A tuple containing all but the head element of the original tuple.
 */
template<typename Head, typename T0, typename... Tail>
std::tuple<T0, Tail...> tuple_tail(std::tuple<Head, T0, Tail...> const & tuple) {
	return tuple_slice<1, sizeof...(Tail) + 1>(tuple);
}

}
