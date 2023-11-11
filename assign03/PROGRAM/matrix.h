#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include "../eigen-3.4.0/Eigen/Dense"

class Matrix
{
    /**
     * This class aim to build the basic block for matrix calculation. It contains:
     * The matrix object (the data stored in Eigen form, the number of row and colume.)
     *
     * Basic operations such as get and set methods.
     *
     * Operations on the matrix object, include:
     * 1. Matrix + Matrix
     * 2. Matrix * constant
     * 3. Matrix * Matrix
     * 4. transpose
     * 5. inverse
     * 6. skew
     * 7. Matrix x Matrix
     * 8. magnitude
     * 9. normalize
     * 10. scale a matrix so that all values are between 0 - 1
     *
     */
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

    // operations described above
    Matrix operator-(const Matrix &other) const;
    Matrix operator+(const Matrix &other) const;
    Matrix operator*(const Matrix &other) const;
    Matrix operator*(const float i) { return Matrix(mat * i); }
    Matrix transpose() const;
    Matrix inverse() const;
    Matrix skew() const;
    Matrix cross(const Matrix &other) const { return skew() * other; }
    float magnitude() const;
    void normalize();

    // return the min and max value of the matrix
    std::tuple<float, float> min_max_scale();

    // scale the matrix so that all values are between 0 - 1.
    void min_max_scale(float max, float min);

    // assign Matrix[row][col] = val
    void assign(int row, int col, float val) { mat(row, col) = val; }

    // get the matrix in Eigen form
    Eigen::MatrixXf get() const { return mat; }

    // get a Matrix[row][col]
    float get_pos(int row, int col) const { return mat(row, col); }

    // get the row number of the matrix
    int get_row() const { return nrow; }

    // get the col number of the matrix
    int get_col() const { return ncol; }

    // convert the matrix as a string
    std::string as_str() const;

    // print the vector.
    void print_str() const;
};

#endif