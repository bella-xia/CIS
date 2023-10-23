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

public:
    Interpolation() : N(0), degree(5) {}
    float ber_poly(int i, int j, int k, Matrix mat);
    void form_row(Matrix mat, int nrow, Matrix &return_mat);
    Matrix assemble_u();
    Matrix assemble_y();
    Matrix interpolate();
};

#endif