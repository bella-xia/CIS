#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "frame.h"
#include <iostream>

class Registration
{
private:
    std::vector<Eigen::MatrixXf> eigens_a, eigens_b;
    int size_a, size_b;
    Eigen::MatrixXf mid_a, mid_b;

public:
    Registration() : eigens_a(std::vector<Eigen::MatrixXf>()), eigens_b(std::vector<Eigen::MatrixXf>()),
                     size_a(0), size_b(0), mid_a(Eigen::MatrixXf(3, 1)), mid_b(Eigen::MatrixXf(3, 1)){};
    bool read_file(std::string input);
    void add_matrix_a(Eigen::MatrixXf eigen)
    {
        eigens_a.push_back(eigen);
        size_a++;
    }
    void add_matrix_b(Eigen::MatrixXf eigen)
    {
        eigens_b.push_back(eigen);
        size_b++;
    }
    void calculate_midpoints();
    Eigen::MatrixXf get_midpoint_a() { return mid_a; }
    Eigen::MatrixXf get_midpoint_b() { return mid_b; }
    Eigen::MatrixXf best_plane_from_points();

private:
};

#endif