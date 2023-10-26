#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include "../eigen-3.4.0/Eigen/Dense"

class Matrix
{

private:
    Eigen::MatrixXf mat;
    int nrow;
    int ncol;

public:
    Matrix();
    Matrix(int dim);
    Matrix(int nrow, int ncol);
    Matrix(Eigen::MatrixXf mat) : mat(mat),
                                  nrow(static_cast<int>(mat.rows())), ncol(static_cast<int>(mat.cols())) {}
    Matrix(float p0, float p1, float p2, float p3, float p4, float p5, float p6, float p7, float p8);
    Matrix(float p0, float p1, float p2);
    Matrix(std::vector<float> &v);

    ~Matrix();

    Matrix operator+(const Matrix &other) const;
    Matrix operator*(const Matrix &other) const;
    Matrix operator*(const float i) { return Matrix(mat * i); }
    Matrix transpose() const;
    Matrix inverse() const;
    Matrix skew() const;
    Matrix cross(const Matrix &other) const { return skew() * other; }
    float magnitude() const;
    void normalize();
    std::tuple<float, float> min_max_scale();
    void min_max_scale(float max, float min);
    void assign(int row, int col, float val) { mat(row, col) = val; }
    Eigen::MatrixXf get() const { return mat; }
    float get_pos(int row, int col) const { return mat(row, col); }
    int get_row() const { return nrow; }
    int get_col() const { return ncol; }

    std::string as_str() const;
    void print_str() const;
};

#endif