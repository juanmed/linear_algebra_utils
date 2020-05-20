#ifndef LINALG_UTILS_V12020_H
#define LINALG_UTILS_V12020_H

#include <iostream>
#include <Eigen/Dense>

inline double clip_scalar(double scalar, double max_value, double min_value)
{
	return  std::fmin(std::fmax(scalar,min_value),max_value);
}

inline double rotation_distance(const Eigen::Matrix3d R1, const Eigen::Matrix3d	R2)
{
	/**
  Measure the angle to rotate from R1 to R2 obtained from
  at http://www.boris-belousov.net/2016/12/01/quat-dist/
	*/
	Eigen::Matrix3d Rd = R1.transpose() * R2;
	double angle = std::acos( (Rd.trace() -1.0) / 2. ); 
	return angle;
}

inline Eigen::Vector3d vex2(const Eigen::Matrix3d R)
{
	Eigen::Vector3d v;
	v << R(2,1), R(0,2), R(1,0);
	return v;
}

inline Eigen::Matrix3d hat(const Eigen::Vector3d v)
{
	Eigen::Matrix3d skew_symmetric = Eigen::Matrix3d::Zero();
	skew_symmetric << 0., -v(2), v(1), 
	v(2), 0., -v(0),
	-v(1), v(0), 0.;
}

inline Eigen::Vector3d matrixToEulerZYX(const Eigen::Matrix3d R)
{
	double roll, pitch, yaw;
	Eigen::Vector3d euler_angles = Eigen::Vector3d::Zero();

	pitch = std::asin(-1.0 * R(2, 0));
	roll = std::atan2(R(2, 1) / std::cos(pitch), R(2, 2) / std::cos(pitch));
  yaw = std::atan2(R(1, 0) / std::cos(pitch), R(0, 0) / std::cos(pitch));

  euler_angles << roll, pitch, yaw;
  return euler_angles;
}

inline Eigen::Matrix3d rotationMatrixDerivative(const Eigen::Matrix3d R,
	const Eigen::Vector3d angular_velocity)
{
	return R * hat(angular_velocity);
}

#endif /* LINALG_UTILS_V12020_H */