#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dr {
	namespace axes {
		inline Eigen::Vector3d x() { return Eigen::Vector3d{1, 0, 0}; };
		inline Eigen::Vector3d y() { return Eigen::Vector3d{0, 1, 0}; };
		inline Eigen::Vector3d z() { return Eigen::Vector3d{0, 0, 1}; };
	}

	inline Eigen::Translation3d translate(Eigen::Vector3d translation) {
		return Eigen::Translation3d{translation};
	}

	inline Eigen::Translation3d translate(double x, double y, double z) {
		return translate(Eigen::Vector3d{x, y, z});
	}

	inline Eigen::AngleAxisd rotate(double angle, Eigen::Vector3d const & axis) {
		return Eigen::AngleAxisd{angle, axis};
	}

	inline Eigen::Isometry3d rotate(double angle, Eigen::Vector3d const & axis, Eigen::Vector3d const & pivot_point) {
		return dr::translate(pivot_point) * Eigen::AngleAxisd{angle, axis} * dr::translate(-pivot_point);
	}

	inline Eigen::AngleAxisd rotate_x(double angle) {
		return dr::rotate(angle, axes::x());
	}

	inline Eigen::Isometry3d rotate_x(double angle, Eigen::Vector3d const & pivot_point) {
		return dr::rotate(angle, axes::x(), pivot_point);
	}

	inline Eigen::AngleAxisd rotate_y(double angle) {
		return dr::rotate(angle, axes::y());
	}

	inline Eigen::Isometry3d rotate_y(double angle, Eigen::Vector3d const & pivot_point) {
		return dr::rotate(angle, axes::y(), pivot_point);
	}

	inline Eigen::AngleAxisd rotate_z(double angle) {
		return dr::rotate(angle, axes::z());
	}

	inline Eigen::Isometry3d rotate_z(double angle, Eigen::Vector3d const & pivot_point) {
		return dr::rotate(angle, axes::z(), pivot_point);
	}
}
