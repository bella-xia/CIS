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

public:
    Registration() : eigens_a(std::vector<Matrix>()), eigens_b(std::vector<Matrix>()),
                     size_a(0), size_b(0), mid_a(Eigen::MatrixXf(3, 1)), mid_b(Eigen::MatrixXf(3, 1)){};
    bool read_file(std::string input);
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
    void calculate_midpoints();
    Matrix get_midpoint_a() { return Matrix(mid_a); }
    Matrix get_midpoint_b() { return Matrix(mid_b); }
    Matrix best_plane_from_points();

private:
};

#endif