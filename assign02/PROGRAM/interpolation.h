#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include "frame.h"

class Interpolation
{

private:
    int N;
    int degree;
    float binomialCoefficients(int n, int k);
    std::vector<Matrix> mats_u;
    std::vector<Matrix> mats_y;
    Matrix coef;
    bool coef_defined;

public:
    Interpolation() : N(0), degree(5), mats_u(std::vector<Matrix>()), mats_y(std::vector<Matrix>()), coef_defined(false) {}
    Interpolation(std::vector<Matrix> u, std::vector<Matrix> y) : N((int)u.size()), degree(5), mats_u(u), mats_y(y), coef_defined(false) {}
    float ber_poly(int i, int j, int k, Matrix mat);
    void form_row(Matrix mat, int nrow, Matrix &return_mat);
    Matrix correction_func(Matrix &real_mat);
    Matrix assemble_u();
    Matrix assemble_y();
    Matrix interpolate();
};

#endif