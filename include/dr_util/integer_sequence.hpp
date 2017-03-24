#include <utility>

namespace dr {

/// Add an offset to an integer sequence.
template<typename T, T offset, T... sequence>
std::integer_sequence<T, (sequence + offset)...> offset_integer_sequence(std::integer_sequence<T, sequence...>) {
	return {};
}

/// Add an offset to an index sequence.
template<std::size_t offset, std::size_t... sequence>
std::index_sequence<(sequence + offset)...> offset_index_sequence(std::index_sequence<sequence...>) {
	return {};
}

}
