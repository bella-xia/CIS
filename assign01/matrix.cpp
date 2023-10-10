#include "matrix.h"

Matrix::Matrix(int nrow, int ncol) : nrow(nrow), ncol(ncol)
{
    mat = Eigen::MatrixXd::Zero(nrow, ncol);
}

Matrix::Matrix() : Matrix(3, 3)
{
}

Matrix::Matrix(int dim) : Matrix(dim, dim)
{
    for (int i = 0; i < dim; i++)
    {
        mat(i, i) = 1.0;
    }
}

Matrix::Matrix(float p0, float p1, float p2, float p3, float p4, float p5, float p6, float p7, float p8) : nrow(3), ncol(3)
{
    mat = Eigen::MatrixXd(nrow, ncol);
    mat << p0, p1, p2,
        p3, p4, p5,
        p6, p7, p8;
}
Matrix::Matrix(float p0, float p1, float p2) : nrow(3), ncol(1)
{
    mat = Eigen::MatrixXd(nrow, ncol);
    mat << p0,
        p1,
        p2;
}

Matrix::Matrix(std::vector<float> &v) : Matrix(v[0], v[1], v[2]) {}

Matrix Matrix::operator+(const Matrix &other) const
{
    return Matrix(mat + other.get());
}

Matrix Matrix::operator*(const Matrix &other) const
{
    return Matrix(mat * other.get());
}

Matrix Matrix::transpose() const
{
    return Matrix(mat.transpose());
}

float Matrix::magnitude() const
{
    return mat.norm();
}

Matrix Matrix::skew() const
{
    if (nrow != 3 || ncol != 1)
    {
        throw std::invalid_argument("Invalid skew operation");
        // TODO: throw error
    }
    Matrix mat_skew = Matrix(3, 3);
    mat_skew.assign(0, 1, -mat(2, 0));
    mat_skew.assign(0, 2, mat(1, 0));
    mat_skew.assign(1, 0, mat(2, 0));
    mat_skew.assign(1, 2, -mat(0, 0));
    mat_skew.assign(2, 0, -mat(1, 0));
    mat_skew.assign(2, 1, mat(0, 0));
    return mat_skew;
}

Matrix Matrix::inverse() const
{
    return Matrix(mat.inverse());
}

std::string Matrix::as_str() const
{
    std::string mat_str = "";
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < ncol; j++)
        {
            mat_str.append(std::to_string(mat(i, j)));
            if (j != ncol - 1)
            {
                mat_str.append(" ");
            }
            else
            {
                mat_str.append("\n");
            }
        }
    }
    mat_str.append("\n");
    return mat_str;
}

void Matrix::print_str() const
{
    std::cout << as_str() << std::endl;
}

Matrix::~Matrix()
{
}