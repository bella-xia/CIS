#include "function.h"

Matrix create_3_3(float p0, float p1, float p2, float p3, float p4, float p5, float p6, float p7, float p8)
{
    Matrix mat3_3 = Matrix(3, 3);
    mat3_3.assign(0, 0, p0);
    mat3_3.assign(0, 1, p1);
    mat3_3.assign(0, 2, p2);
    mat3_3.assign(1, 0, p3);
    mat3_3.assign(1, 1, p4);
    mat3_3.assign(1, 2, p5);
    mat3_3.assign(2, 0, p6);
    mat3_3.assign(2, 1, p7);
    mat3_3.assign(2, 2, p8);
    return mat3_3;
}

Matrix create_3_1(float p0, float p1, float p2)
{
    Matrix mat3_1 = Matrix(3, 1);

    mat3_1.assign(0, 0, p0);
    mat3_1.assign(1, 0, p1);
    mat3_1.assign(2, 0, p2);
    return mat3_1;
}
Matrix convert_from_eigen(Eigen::MatrixXf eigen)
{
    int r = static_cast<int>(eigen.rows());
    int c = static_cast<int>(eigen.cols());
    Matrix mat = Matrix(r, c);
    for (int i = 0; i < r; i++)
    {
        for (int j = 0; j < c; j++)
        {
            mat.assign(i, j, eigen(i, j));
        }
    }
    return mat;
}

Eigen::MatrixXf convert_to_eigen(Matrix mat)
{
    int nrow = mat.get_row();
    int ncol = mat.get_col();
    Eigen::MatrixXf eigen_mat(nrow, ncol);
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < ncol; j++)
        {
            eigen_mat(i, j) = mat.get_pos(i, j);
        }
    }
    return eigen_mat;
}