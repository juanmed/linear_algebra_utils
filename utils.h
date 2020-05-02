#ifndef LINALG_UTILS_V12020_H
#define LINALG_UTILS_V12020_H

#include <iostream>
#include <Eigen/Dense>

inline double clip_scalar(double scalar, double max_value, double min_value)
{
	return  std::fmin(std::fmax(scalar,min_value),max_value);
}

#endif /* LINALG_UTILS_V12020_H */