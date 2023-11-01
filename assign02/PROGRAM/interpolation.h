#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include "frame.h"

class Interpolation
{
/* This class aims to produce a distortion correction function to convert
 * the measured data points to the real data points.
 * The function is in the format of: 
 * |               ：                    |   |  c_{000}  |    | : |
 * |F_{000}(u) F_{100}(u) ... F_{555}(u)| *  |  c_{100}  |  = | y |
 * |               ：                    |   |    :      |    | : |
 *                                           |  c_{555}  |
 * where F_{ijk}(u) is B(i,u_x) * B(j, u_y) * B(k, u_z)
 * where B(a, b) = C(degree, a) * (1 - b) ^ (degree - a) * b ^ a 
*/

private:
    int N; // number of data points
    int degree; // degree used in bernstein polynomial
    float max, min; // the max and min of all data points
    std::vector<Matrix> mats_u; // matrix to store the real data
    std::vector<Matrix> mats_y; // matrix to store the expected data
    Matrix coef; // matrix to store the coeffcient matrix
    bool coef_defined, max_min_defined; // boolean to check whether coef is defined

    // calculate the binomial coefficient C(n,k)
    float binomialCoefficients(int n, int k);

public:
    // default constructor to create an empty interpolation class
    Interpolation() : N(0), degree(5), mats_u(std::vector<Matrix>()), mats_y(std::vector<Matrix>()),
                      coef_defined(false), max_min_defined(false) {}
    // constructor to create an interpolation class with given u and given y
    Interpolation(std::vector<Matrix> u, std::vector<Matrix> y) : N((int)u.size()), degree(5), mats_u(u), mats_y(y),
                                                                  coef_defined(false), max_min_defined(false) {}
    // compute the F{i, j, k} (mat)
    float ber_poly(int i, int j, int k, Matrix mat);

    // given a single mat matrix, create a raw of the F(mat) * c = y descibed above.
    void form_row(Matrix mat, int nrow, Matrix &return_mat);
    
    // given a measured matrix, output the expected data by computing the distortion correction function
    Matrix correction_func(Matrix real_mat);

    // create the left-most F(u) matrix
    Matrix assemble_u();

    // create the right-most y matrix
    Matrix assemble_y();

    // calculate the c matrix.
    Matrix interpolate();
};

#endif