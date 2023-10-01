#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Dense>

class Matrix
{

private:
    std::string name;
    std::vector<std::vector<float>> mat;
    int nrow;
    int ncol;

public:
    Matrix();
    Matrix(int dim);
    Matrix(int nrow, int ncol);
    Matrix(std::vector<std::vector<float>> mat) : name("taylor"), mat(mat), nrow(mat.size()), ncol(mat[0].size()) {}

    ~Matrix();

    Matrix operator+(const Matrix &other) const;
    Matrix operator*(const Matrix &other) const;
    Matrix transpose() const;
    Matrix inverse() const;
    Matrix skew() const;
    Matrix cross(const Matrix &other) const { return skew() * other; }
    float magnitude() const;

    void assign(int row, int col, float val) { mat[row][col] = val; }

    std::vector<std::vector<float>> get() const { return mat; }
    float get_pos(int row, int col) const { return mat[row][col]; }
    int get_row() const { return nrow; }
    int get_col() const { return ncol; }

    std::string asstr() const;
};

#endif