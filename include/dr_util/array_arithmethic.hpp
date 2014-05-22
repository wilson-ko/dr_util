#include <array>
#include <utility>
#include <algorithm>

/**
 * This header allows general arithmetic operations on std::arrays.
 * You are strongly discouraged to include this header in you own headers,
 * as doing so will leak operators for std:array into every translation unit
 * that includes your header.
 */

namespace dr_util {

	/// Scalar multiplication.
	template<typename T1, typename T2, std::size_t N>
	std::array<decltype(std::declval<T1>() * std::declval<T2>()), N>
	operator * (std::array<T1, N> array, T2 const & scalar) {
		for (auto & elem : array) elem = elem * scalar;
		return array;
	}

	/// Scalar division.
	template<typename T1, typename T2, std::size_t N>
	std::array<decltype(std::declval<T1>() / std::declval<T2>()), N>
	operator / (std::array<T1, N> array, T2 const & scalar) {
		for (auto & elem : array) elem = elem / scalar;
		return array;
	}

	/// Scalar modulo.
	template<typename T1, typename T2, std::size_t N>
	std::array<decltype(std::declval<T1>() % std::declval<T2>()), N>
	operator % (std::array<T1, N> array, T2 const & scalar) {
		for (auto & elem : array) elem = elem % scalar;
		return array;
	}

	/// Element-wise addition.
	template<typename T1, typename T2, std::size_t N>
	std::array<decltype(std::declval<T1>() + std::declval<T2>()), N>
	operator + (std::array<T1, N> a1, std::array<T2, N> const & a2) {
		for (std::size_t i = 0; i < N; ++i) {
			a1[i] = a1[i] + a2[i];
		}
		return a1;
	}

	/// Element-wise subtraction.
	template<typename T1, typename T2, std::size_t N>
	std::array<decltype(std::declval<T1>() - std::declval<T2>()), N>
	operator - (std::array<T1, N> a1, std::array<T2, N> const & a2) {
		for (std::size_t i = 0; i < N; ++i) {
			a1[i] = a1[i] - a2[i];
		}
		return a1;
	}

	/// Check if all elements in a container evaluate to false.
	template<typename T>
	bool all_zero(T const & container) {
		return std::all_of(container.begin(), container.end(), [] (typename T::value_type const & x) { return !x; });
	}

	/// Check if all elements in a container evaluate to false.
	template<typename T>
	bool all_nonzero(T const & container) {
		return std::all_of(container.begin(), container.end(), [] (typename T::value_type const & x) { return !!x; });
	}

}
