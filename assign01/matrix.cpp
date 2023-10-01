#include "matrix.h"
#include <iostream>

Matrix::Matrix(int nrow, int ncol) : name("hisashi"), nrow(nrow), ncol(ncol)
{
    for (int i = 0; i < nrow; i++)
    {
        std::vector<float> dim(ncol, 0.0);
        mat.push_back(dim);
    }
}

Matrix::Matrix() : Matrix(3, 3)
{
}

Matrix::Matrix(int dim) : Matrix(dim, dim)
{
    for (int i = 0; i < dim; i++)
    {
        mat[i][i] = 1.0;
    }
}

Matrix Matrix::operator+(const Matrix &other) const
{
    if (nrow != other.get_row() || ncol != other.get_col())
    {
        throw std::invalid_argument("Matrix column and row doesn't match for add operation");
        // TODO: throw error
    }

    Matrix add_mat = Matrix(nrow, ncol);
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < ncol; j++)
        {
            add_mat.assign(i, j, mat[i][j] + other.get_pos(i, j));
        }
    }
    return add_mat;
}

Matrix Matrix::operator*(const Matrix &other) const
{
    if (ncol != other.get_row())
    {
        throw std::invalid_argument("First matrix column and second matrix row doesn't match for dot operation");
        // TODO: throw error
    }
    Matrix dot_mat = Matrix(nrow, other.get_col());
    for (int i = 0; i < nrow; ++i)
    {
        for (int j = 0; j < other.get_col(); ++j)
        {
            float value = 0.0;
            for (int k = 0; k < ncol; ++k)
            {
                value += mat[i][k] * other.get_pos(k, j);
            }
            dot_mat.assign(i, j, value);
        }
    }
    return dot_mat;
}

Matrix Matrix::transpose() const
{
    Matrix trans_mat = Matrix(ncol, nrow);
    for (int i = 0; i < ncol; ++i)
    {
        for (int j = 0; j < nrow; ++j)
        {
            trans_mat.assign(i, j, mat[j][i]);
        }
    }
    return trans_mat;
}

float Matrix::magnitude() const
{
    if (ncol != 1 && nrow != 1)
    {
        throw std::invalid_argument("Unable to calculate magnitude.");
        // TODO: throw error
    }
    float sum = 0.0;
    if (ncol == 1)
    {
        for (int i = 0; i < nrow; i++)
        {
            sum += mat[i][0] * mat[i][0];
        }
    }
    else
    {
        for (int i = 0; i < ncol; i++)
        {
            sum += mat[0][i] * mat[0][i];
        }
    }
    return sqrt(sum);
}

Matrix Matrix::skew() const
{
    if (nrow != 3 || ncol != 1)
    {
        throw std::invalid_argument("Invalid skew operation");
        // TODO: throw error
    }
    Matrix mat_skew = Matrix(3, 3);
    mat_skew.assign(0, 1, -mat[2][0]);
    mat_skew.assign(0, 2, mat[1][0]);
    mat_skew.assign(1, 0, mat[2][0]);
    mat_skew.assign(1, 2, -mat[0][0]);
    mat_skew.assign(2, 0, -mat[1][0]);
    mat_skew.assign(2, 1, mat[0][0]);
    return mat_skew;
}

std::string Matrix::asstr() const
{
    std::string mat_str = name + "\n" + "[";
    for (int i = 0; i < nrow; i++)
    {
        mat_str.append("[");
        for (int j = 0; j < ncol; j++)
        {
            mat_str.append(std::to_string(mat[i][j]));
            if (j != ncol - 1)
            {
                mat_str.append(", ");
            }
            else
            {
                mat_str.append("]\n");
            }
        }
    }
    mat_str.append("]\n");
    return mat_str;
}
Matrix::~Matrix()
{
}