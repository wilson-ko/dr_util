#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>


extern template class Eigen::Matrix<double, 3, 1>;
extern template class Eigen::Matrix<double, 3, 3>;

namespace dr {
	namespace axes {
		inline Eigen::Vector3d x() { return Eigen::Vector3d{1, 0 , 0}; };
		inline Eigen::Vector3d y() { return Eigen::Vector3d{0, 1 , 0}; };
		inline Eigen::Vector3d z() { return Eigen::Vector3d{0, 0 , 1}; };
	}

	inline Eigen::Matrix3d rotate_x(double angle) {
		Eigen::AngleAxis<double> result{angle, axes::x()};
		return result.matrix();
	}

	inline Eigen::Matrix3d rotate_y(double angle) {
		Eigen::AngleAxis<double> result{angle, axes::y()};
		return result.matrix();
	}

	inline Eigen::Matrix3d rotate_z(double angle) {
		Eigen::AngleAxis<double> result{angle, axes::z()};
		return result.matrix();
	}
}
