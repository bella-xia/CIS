#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "frame.h"
#include <iostream>

class Registration
{
private:
    std::vector<Matrix> eigens_a, eigens_b;
    int size_a, size_b;
    Eigen::MatrixXf mid_a, mid_b;
    bool a_mid_calculated;

public:
    Registration() : eigens_a(std::vector<Matrix>()), eigens_b(std::vector<Matrix>()),
                     size_a(0), size_b(0), mid_a(Eigen::MatrixXf(3, 1)), mid_b(Eigen::MatrixXf(3, 1)),
                     a_mid_calculated(false){};
    void add_matrix_a(Matrix eigen)
    {
        eigens_a.push_back(eigen);
        size_a++;
    }
    void add_matrix_b(Matrix eigen)
    {
        eigens_b.push_back(eigen);
        size_b++;
    }
    void clean_matrix_b()
    {
        eigens_b.clear();
        size_b = 0;
    }
    void clean_matrix_a()
    {
        eigens_a.clear();
        size_a = 0;
        a_mid_calculated = false;
    }

    void calculate_midpoint_a();
    void calculate_midpoint_b();
    void get_matrix_a_from_b();
    Matrix get_midpoint_a() { return Matrix(mid_a); }
    Matrix get_midpoint_b() { return Matrix(mid_b); }
    Frame point_cloud_registration();
    Matrix pivot_calibration(const std::vector<Frame> &f);
    void pass_matrix(Eigen::MatrixXf &rot_mat, Eigen::MatrixXf &pos_mat, const std::vector<Frame> &f);

private:
};

#endif