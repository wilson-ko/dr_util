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

	inline Eigen::Affine3d rotate_x(double angle) {
		return Eigen::Affine3d{Eigen::AngleAxis<double>{angle, axes::x()}};
	}

	inline Eigen::Affine3d rotate_y(double angle) {
		return Eigen::Affine3d{Eigen::AngleAxis<double>{angle, axes::y()}};
	}

	inline Eigen::Affine3d rotate_z(double angle) {
		return Eigen::Affine3d{Eigen::AngleAxis<double>{angle, axes::z()}};
	}

	inline Eigen::Affine3d translate(Eigen::Vector3d translation) {
		return Eigen::Affine3d{Eigen::Translation3d{translation}};
	}

	inline Eigen::Affine3d translate(double x, double y, double z) {
		return translate(Eigen::Vector3d{x, y, z});
	}
}
