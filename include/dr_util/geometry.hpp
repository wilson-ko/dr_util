#pragma once

#include <cmath>

namespace dr {

/// Get the value of pi as double.
inline constexpr double pi() { return 3.1415926535897932384626433832795028841971693993751058; }

/// Convert degrees to radians.
inline constexpr double degrees(double angle) { return angle * pi() / 180.0; }

/// Convert radians to degrees.
inline constexpr double to_degrees(double angle) { return angle * 180.0 / pi(); }

/// Get the sign of a number.
template<typename T>
inline constexpr int sign(T const & value) { return (value > 0) - (value < 0); }

/// Get an angle in the [0, 2*Pi) range.
inline double normalizeAngle(double angle) {
	return std::fmod(angle, 2 * pi()) + (angle < 0) * 2 * pi();
}

/// Return an angle in a specific range.
inline double angleFrom(double angle, double range_start) {
	return normalizeAngle(angle) + int(range_start / 2 / pi()) * 2 * pi()
		+ (sign(range_start) ==  1 && normalizeAngle(angle) <  normalizeAngle(range_start)) *  2 * pi()
		+ (sign(range_start) == -1 && normalizeAngle(angle) >= normalizeAngle(range_start)) * -2 * pi();
}

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
