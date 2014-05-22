#pragma once

#include <cmath>

namespace dr {

/// Get the value of pi as double.
inline constexpr double pi() { return std::atan(1)*4; }

/// Convert degrees to radians.
inline constexpr double degrees(double angle) { return angle * pi() / 180.0; }

/// Calculate the square of a number.
/**
 * \param a The number to square.
 * \return a squared.
 */
inline constexpr double square(double a) { return a * a; }

/// Calculate the sum of the squares of the arguments.
/**
 * \param args The numbers to square.
 * \return The sum of the squares.
 */
template<typename... Args> constexpr double square_sum(Args... args);

template<typename... Args>
constexpr double square_sum(double head, Args... tail) {
	return square(head) + square_sum(tail...);
}

template<>
constexpr double square_sum() {
	return 0;
}

/// Calculate the length of a vector.
/**
 * \param args The vector coordinates.
 * \return The length of the point vector.
 */
template<typename... Args>
constexpr double length(Args... args) {
	return std::sqrt(square_sum(args...));
}

/// Calculate one angle of a triangle.
/**
 * \param a The length of side a.
 * \param b The length of side b.
 * \param c The length of side c.
 * \return The angle opposite of side a.
 */
inline double angle(double a, double b, double c) {
	return std::acos((b*b + c*c - a*a) / (2 * b * c));
}

}
