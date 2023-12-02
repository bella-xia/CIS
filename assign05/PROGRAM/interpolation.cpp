#include "interpolation.h"

float Interpolation::binomialCoefficients(int n, int k)
{
    if (k == 0 || k == n)
        return 1;
    return binomialCoefficients(n - 1, k - 1) + binomialCoefficients(n - 1, k);
}

float Interpolation::ber_poly(int i, int j, int k, Matrix mat)
{
    float x = mat.get_pos(0, 0);
    float y = mat.get_pos(1, 0);
    float z = mat.get_pos(2, 0);

    //B(i, x) = C(degree, i) * (1 - x) ^ (degree - i) * x ^ i 
    float B_x = binomialCoefficients(degree, i) * pow((1 - x), degree - i) * pow(x, i);

    //B(j, y) = C(degree, j) * (1 - y) ^ (degree - j) * y ^ j 
    float B_y = binomialCoefficients(degree, j) * pow((1 - y), degree - j) * pow(y, j);

    //B(k, z) = C(degree, k) * (1 - z) ^ (degree - k) * z ^ k
    float B_z = binomialCoefficients(degree, k) * pow((1 - z), degree - k) * pow(z, k);
    return B_x * B_y * B_z;
}

void Interpolation::form_row(Matrix mat, int nrow, Matrix &return_mat)
{
    for (int i = 0; i <= degree; i++)
    {
        for (int j = 0; j <= degree; ++j)
        {
            for (int k = 0; k <= degree; ++k)
            {
                float ber_poly_val = ber_poly(i, j, k, mat);
                return_mat.assign(nrow, i * pow(degree + 1, 2) + j * (degree + 1) + k,
                                  ber_poly_val);
            }
        }
    }
}

Matrix Interpolation::assemble_u()
{
    Matrix norm_u_mats = Matrix(N, 3);
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            norm_u_mats.assign(i, j, mats_u.at(i).get_pos(j, 0));
        }
    }
    //scale the norm_u_mats to so that all its data are between 0 - 1
    std::tuple<float, float> max_min_val = norm_u_mats.min_max_scale();
    max = std::get<0>(max_min_val);
    min = std::get<1>(max_min_val);
    max_min_defined = true;
    Matrix return_mat = Matrix(N, pow(degree + 1, 3));
    for (int i = 0; i < N; ++i)
    {
        form_row(Matrix(norm_u_mats.get_pos(i, 0), norm_u_mats.get_pos(i, 1), norm_u_mats.get_pos(i, 2)), i, return_mat);
    }
    return return_mat;
}

Matrix Interpolation::assemble_y()
{
    Matrix return_mat = Matrix(N, 3);
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            return_mat.assign(i, j, mats_y.at(i).get_pos(j, 0));
        }
    }
    return return_mat;
}

Matrix Interpolation::interpolate()
{
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> u_mat(assemble_u().get());
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> y_mat(assemble_y().get());
    Matrix coefs = Matrix(u_mat.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y_mat));
    coef = Matrix(coefs.get());
    coef_defined = true;
    return coefs;
}

Matrix Interpolation::correction_func(Matrix real_mat)
{
    assert(coef_defined);
    assert(max_min_defined);
    real_mat.min_max_scale(max, min);
    Matrix return_mat = Matrix(1, pow(degree + 1, 3));
    form_row(real_mat, 0, return_mat);
    Matrix y(return_mat * coef);
    return y;
}