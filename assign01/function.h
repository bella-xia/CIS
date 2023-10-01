#ifndef FUNCTION_H
#define FUNCTION_H

#include "matrix.h"
#include <Eigen/Dense>

Matrix create_3_3(float p0, float p1, float p2, float p3, float p4, float p5, float p6, float p7, float p8);
Matrix create_3_1(float p0, float p1, float p2);
Matrix convert_from_eigen(Eigen::MatrixXf eigen);
Eigen::MatrixXf convert_to_eigen(Matrix mat);
#endif